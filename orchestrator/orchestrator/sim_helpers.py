#!/usr/bin/env python3
"""
Sim Helpers — simulation utility services for perception/harvest testing.

Currently exposes:

  /sim/spawn_at_cluster (std_srvs/Trigger)
    Teleports the Husky model in Gazebo to a scout pose in front of the
    cluster specified by the `cluster_id` parameter. Useful for static
    per-cluster perception tests without going through navigation.

    Uses Gazebo's /world/<world>/set_pose service (via `ign service`)
    to instantly relocate the robot.

    NOTE on TF: this teleport does NOT update the launch's static
    world→odom transform, so TF-dependent code (depth_processor 3D
    backprojection) will be off after teleport. For 2D YOLO testing
    (current phase) this is fine — image topics carry their own
    camera_optical_frame data. A dynamic world→odom broadcaster will be
    added here when we wire up the 3D positioning phase.

Parameters:
  cluster_id              : 'tree_000'        — which cluster to teleport in front of
  scout_y                 : 4.85              — aisle Y for scout pose
  scout_yaw               : -pi/2             — Husky yaw (faces -Y → toward tree row)
  spawn_z                 : 0.0               — ground level Z
  gz_world_name           : 'orchard'
  husky_model_name        : 'husky_robocot'
  tree_positions_yaml     : ''                — defaults to robot_arm/config/orchard_tree_positions.yaml

Usage:
  ros2 run orchestrator sim_helpers

  ros2 param set /sim_helpers cluster_id tree_005
  ros2 service call /sim/spawn_at_cluster std_srvs/srv/Trigger '{}'
"""

from __future__ import annotations

import math
import os
import shutil
import subprocess
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node

import yaml
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """Yaw-only rotation → quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SimHelpers(Node):

    def __init__(self):
        super().__init__('sim_helpers')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('cluster_id', 'tree_000')
        self.declare_parameter('scout_y', 4.85)
        self.declare_parameter('scout_yaw', -math.pi / 2)
        self.declare_parameter('spawn_z', 0.0)
        self.declare_parameter('gz_world_name', 'orchard')
        self.declare_parameter('husky_model_name', 'husky_robocot')
        self.declare_parameter('tree_positions_yaml', '')

        # ── Tree positions (ground truth) ───────────────────────
        self._tree_positions: Dict[str, Tuple[float, float]] = (
            self._load_tree_positions())

        # ── Service ─────────────────────────────────────────────
        self.create_service(
            Trigger, '/sim/spawn_at_cluster', self._on_spawn)

        self.get_logger().info('=' * 60)
        self.get_logger().info('SIM HELPERS ready')
        self.get_logger().info(
            f'  trees loaded: {len(self._tree_positions)}')
        self.get_logger().info(
            f'  default cluster_id: {self.get_parameter("cluster_id").value}')
        self.get_logger().info(
            f'  scout_y={self.get_parameter("scout_y").value:.2f}, '
            f'scout_yaw={math.degrees(self.get_parameter("scout_yaw").value):.0f}°')
        self.get_logger().info(
            f'  husky model: {self.get_parameter("husky_model_name").value}')
        self.get_logger().info(
            f'  world: {self.get_parameter("gz_world_name").value}')
        self.get_logger().info('  service: /sim/spawn_at_cluster')
        self.get_logger().info('=' * 60)

    # ─── Tree positions ─────────────────────────────────────────

    def _load_tree_positions(self) -> Dict[str, Tuple[float, float]]:
        path = self.get_parameter('tree_positions_yaml').value
        if not path:
            try:
                share = get_package_share_directory('robot_arm')
                path = os.path.join(
                    share, 'config', 'orchard_tree_positions.yaml')
            except Exception:
                path = ''
        if not path or not os.path.isfile(path):
            self.get_logger().warn(f'tree positions yaml not found: {path}')
            return {}
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            trees = data.get('trees', []) or []
            return {t['id']: (float(t['x']), float(t['y'])) for t in trees}
        except Exception as e:
            self.get_logger().error(f'Failed to read {path}: {e}')
            return {}

    # ─── Gazebo set_pose teleport ───────────────────────────────

    def _gz_set_pose(self, model_name: str,
                     x: float, y: float, z: float,
                     qx: float = 0.0, qy: float = 0.0,
                     qz: float = 0.0, qw: float = 1.0) -> bool:
        world = self.get_parameter('gz_world_name').value
        srv = f'/world/{world}/set_pose'
        req_txt = (
            f'name: "{model_name}"\n'
            f'position {{ x: {x} y: {y} z: {z} }}\n'
            f'orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }}\n'
        )
        cli_args = [
            'service',
            '-s', srv,
            '--reqtype', 'ignition.msgs.Pose',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2500',
            '--req', req_txt,
        ]
        for exe in ('ign', 'gz'):
            exe_path = shutil.which(exe)
            if not exe_path:
                continue
            try:
                ret = subprocess.run(
                    [exe_path] + cli_args,
                    capture_output=True, text=True,
                    timeout=6.0, check=False)
                out = (ret.stdout or '') + (ret.stderr or '')
                if ret.returncode == 0 and 'true' in out.lower():
                    return True
                self.get_logger().warn(
                    f'[GZ] {exe} set_pose rc={ret.returncode}: {out[:200]}')
            except Exception as e:
                self.get_logger().warn(f'[GZ] {exe} subprocess: {e}')
        self.get_logger().error('[GZ] No ign/gz on PATH; teleport disabled.')
        return False

    # ─── Service callback ───────────────────────────────────────

    def _on_spawn(self, request, response):
        cid = self.get_parameter('cluster_id').value
        pos = self._tree_positions.get(cid)
        if pos is None:
            response.success = False
            response.message = (
                f'Unknown cluster_id "{cid}". '
                f'Known: {list(self._tree_positions.keys())[:5]}...')
            self.get_logger().error(response.message)
            return response

        tx, _ty = pos
        sy = float(self.get_parameter('scout_y').value)
        sz = float(self.get_parameter('spawn_z').value)
        syaw = float(self.get_parameter('scout_yaw').value)
        qx, qy, qz, qw = _yaw_to_quat(syaw)

        # Scout pose: X at the tree's X, Y at aisle, yaw faces tree row.
        husky = self.get_parameter('husky_model_name').value
        ok = self._gz_set_pose(husky, tx, sy, sz, qx, qy, qz, qw)
        if ok:
            response.success = True
            response.message = (
                f'Teleported {husky} → {cid} scout '
                f'({tx:.2f}, {sy:.2f}, {sz:.2f}, yaw={math.degrees(syaw):.0f}°)')
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f'set_pose failed for {husky}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimHelpers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
