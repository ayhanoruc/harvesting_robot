#!/usr/bin/env python3
"""
Harvest Orchestrator — autonomous multi-cluster harvest demo.

Drives Husky through a sequence of cluster scout poses on /cmd_vel (mimicking
the open-loop behavior of wasd_teleop, but closed-loop using TF feedback),
triggering simple_cluster_harvester at each scout pose.

All cluster positions are ground-truth (loaded from
robot_arm/config/orchard_tree_positions.yaml). All boll positions are
ground-truth (handled inside simple_cluster_harvester via orchard_bolls.yaml).

Pipeline per cluster:
  1. drive_to(scout_x, scout_y, scout_yaw)  — P-controller on cmd_vel,
     pose feedback from TF world←husky_base_link.
  2. SetParameter tree_id=<cluster> on simple_cluster_harvester.
  3. Call /simple_harvest/start (Trigger). Wait until done.
  4. Continue to next cluster.

Topics:
  pub /cmd_vel              (geometry_msgs/Twist)
  pub /harvest_orch/status  (std_msgs/String)
  TF  world ← husky_base_link

Service:
  /harvest/start  (std_srvs/Trigger) — start the multi-cluster sequence

Parameters:
  cluster_sequence : list[str]   default ['tree_000','tree_001','tree_002']
  scout_y          : float       aisle Y for scout poses           (4.85)
  scout_yaw        : float       Husky yaw at scout                (0.0)
  pos_tol          : float       distance tolerance (m)            (0.20)
  yaw_tol          : float       yaw tolerance (rad)               (0.10)
  max_lin          : float       max linear vel (m/s)              (0.5)
  max_ang          : float       max angular vel (rad/s)           (0.6)
  drive_timeout_s  : float       per-waypoint drive timeout        (90.0)

Usage:
  ros2 run orchestrator harvest_orchestrator
  ros2 service call /harvest/start std_srvs/srv/Trigger '{}'
"""

from __future__ import annotations

import math
import os
import time
from typing import Optional, Tuple

import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import yaml
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory


def _yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


class HarvestOrchestrator(Node):

    def __init__(self):
        super().__init__('harvest_orchestrator')
        self.cb = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('cluster_sequence',
                               ['tree_000', 'tree_001', 'tree_002'])
        self.declare_parameter('scout_y', 4.85)
        self.declare_parameter('scout_yaw', 0.0)
        self.declare_parameter('pos_tol', 0.20)
        self.declare_parameter('yaw_tol', 0.10)
        self.declare_parameter('max_lin', 0.5)
        self.declare_parameter('max_ang', 0.6)
        self.declare_parameter('drive_timeout_s', 90.0)
        self.declare_parameter('tree_positions_yaml', '')

        # ── State ───────────────────────────────────────────────
        self._busy = False

        # ── Tree positions (ground truth) ───────────────────────
        self._tree_positions = self._load_tree_positions()

        # ── TF ──────────────────────────────────────────────────
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True)

        # ── Pubs ────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(
            String, '/harvest_orch/status', 10)

        # ── Clients (simple_cluster_harvester) ──────────────────
        self.harvest_set_params_cli = self.create_client(
            SetParameters,
            '/simple_cluster_harvester/set_parameters',
            callback_group=self.cb)
        self.harvest_start_cli = self.create_client(
            Trigger, '/simple_harvest/start', callback_group=self.cb)

        # ── Service ─────────────────────────────────────────────
        self.create_service(
            Trigger, '/harvest/start', self._on_start,
            callback_group=self.cb)

        seq = list(self.get_parameter('cluster_sequence').value)
        self.get_logger().info('=' * 60)
        self.get_logger().info('HARVEST ORCHESTRATOR ready')
        self.get_logger().info(f'  sequence: {seq}')
        self.get_logger().info(
            f'  scout_y={self.get_parameter("scout_y").value:.2f}, '
            f'scout_yaw={self.get_parameter("scout_yaw").value:.2f} rad')
        self.get_logger().info(
            f'  loaded {len(self._tree_positions)} tree positions')
        for tid in seq:
            p = self._tree_positions.get(tid)
            if p:
                self.get_logger().info(
                    f'    {tid}: ({p[0]:.2f}, {p[1]:.2f})')
            else:
                self.get_logger().warn(f'    {tid}: MISSING from yaml!')
        self.get_logger().info('  service: /harvest/start')
        self.get_logger().info('=' * 60)

    # ─── Tree positions ─────────────────────────────────────────

    def _load_tree_positions(self) -> dict:
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

    # ─── TF helper ──────────────────────────────────────────────

    def _get_husky_pose(self) -> Optional[Tuple[float, float, float]]:
        """Return Husky (x, y, yaw) in world frame, or None on failure."""
        try:
            t = self.tf_buffer.lookup_transform(
                'world', 'husky_base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(f'TF world←husky_base_link: {e}')
            return None

    # ─── Drive controller ──────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _drive_to(self, target_x, target_y, target_yaw) -> bool:
        """Closed-loop drive to (x, y, yaw) using P-control on cmd_vel.

        Three regimes:
          1. Far from goal & misaligned with heading → rotate in place.
          2. Far from goal & aligned        → drive forward, small yaw correction.
          3. At goal position               → rotate to target_yaw, then stop.
        """
        pos_tol = float(self.get_parameter('pos_tol').value)
        yaw_tol = float(self.get_parameter('yaw_tol').value)
        max_lin = float(self.get_parameter('max_lin').value)
        max_ang = float(self.get_parameter('max_ang').value)
        timeout = float(self.get_parameter('drive_timeout_s').value)

        rate_hz = 10.0
        period = 1.0 / rate_hz
        t0 = time.time()
        last_log = 0.0

        while time.time() - t0 < timeout:
            pose = self._get_husky_pose()
            if pose is None:
                time.sleep(period)
                continue
            cx, cy, cyaw = pose
            dx = target_x - cx
            dy = target_y - cy
            dist = math.hypot(dx, dy)

            now = time.time()
            if now - last_log > 1.0:
                self.get_logger().info(
                    f'  drive: ({cx:6.2f},{cy:6.2f}) yaw={math.degrees(cyaw):+6.1f}°  '
                    f'→ ({target_x:.2f},{target_y:.2f}) yaw={math.degrees(target_yaw):+5.1f}°  '
                    f'dist={dist:.2f}m')
                last_log = now

            cmd = Twist()
            if dist < pos_tol:
                # Stage 3: rotate to final yaw
                yaw_err = _wrap_pi(target_yaw - cyaw)
                if abs(yaw_err) < yaw_tol:
                    self._stop()
                    self.get_logger().info(
                        f'  drive done: ({cx:.3f},{cy:.3f}) '
                        f'yaw={math.degrees(cyaw):+.1f}°')
                    return True
                cmd.angular.z = _clip(2.0 * yaw_err, -max_ang, max_ang)
                cmd.linear.x = 0.0
            else:
                heading = math.atan2(dy, dx)
                yaw_err = _wrap_pi(heading - cyaw)
                if abs(yaw_err) > 0.30:
                    # Stage 1: turn in place to face heading
                    cmd.angular.z = _clip(2.0 * yaw_err, -max_ang, max_ang)
                    cmd.linear.x = 0.0
                else:
                    # Stage 2: drive forward, P-correct yaw
                    cmd.linear.x = _clip(0.8 * dist, 0.05, max_lin)
                    cmd.angular.z = _clip(
                        1.5 * yaw_err, -0.5 * max_ang, 0.5 * max_ang)
            self.cmd_pub.publish(cmd)
            time.sleep(period)

        self._stop()
        self.get_logger().error(f'drive timeout (>{timeout:.0f}s)')
        return False

    # ─── Simple-harvester triggers ──────────────────────────────

    def _set_harvester_tree(self, tree_id: str) -> bool:
        if not self.harvest_set_params_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'simple_cluster_harvester set_parameters unavailable — '
                'is the node running?')
            return False
        req = SetParameters.Request(parameters=[
            Parameter(
                name='tree_id',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=tree_id))])
        future = self.harvest_set_params_cli.call_async(req)
        self._wait_future(future, 5.0)
        if future.result() is None:
            return False
        return all(r.successful for r in future.result().results)

    def _trigger_harvest(self) -> bool:
        if not self.harvest_start_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '/simple_harvest/start unavailable — '
                'is simple_cluster_harvester running?')
            return False
        future = self.harvest_start_cli.call_async(Trigger.Request())
        # Per-cluster pick can take minutes (6 bolls × ~40s)
        result = self._wait_future(future, 1200.0)
        if result is None:
            return False
        if not result.success:
            self.get_logger().warn(f'harvest reported failure: {result.message}')
        else:
            self.get_logger().info(f'harvest done: {result.message}')
        return result.success

    def _wait_future(self, future, timeout_sec: float):
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout_sec:
                return None
            time.sleep(0.05)
        return future.result()

    # ─── Status ─────────────────────────────────────────────────

    def _publish_status(self, msg: str):
        self.status_pub.publish(String(data=msg))
        self.get_logger().info(f'[STATUS] {msg}')

    # ─── Main entry ─────────────────────────────────────────────

    def _on_start(self, request, response):
        if self._busy:
            response.success = False
            response.message = 'Already running'
            return response

        sequence = list(self.get_parameter('cluster_sequence').value)
        scout_y = float(self.get_parameter('scout_y').value)
        scout_yaw = float(self.get_parameter('scout_yaw').value)

        self._busy = True
        t_total = time.time()
        completed = 0
        failed_drives = 0
        failed_picks = 0

        try:
            for i, tree_id in enumerate(sequence, 1):
                pos = self._tree_positions.get(tree_id)
                if pos is None:
                    self.get_logger().warn(
                        f'[{i}/{len(sequence)}] unknown tree_id={tree_id}, skip')
                    continue
                tx, _ty = pos
                # Scout: same X as tree (so arm faces tree sideways), aisle Y.
                sx, sy = tx, scout_y

                self._publish_status(
                    f'[{i}/{len(sequence)}] driving to {tree_id} scout '
                    f'({sx:.2f}, {sy:.2f}, yaw={math.degrees(scout_yaw):.0f}°)')
                if not self._drive_to(sx, sy, scout_yaw):
                    self._publish_status(
                        f'[{i}/{len(sequence)}] drive to {tree_id} FAILED')
                    failed_drives += 1
                    continue

                self._publish_status(
                    f'[{i}/{len(sequence)}] reached {tree_id} → harvesting')
                if not self._set_harvester_tree(tree_id):
                    self.get_logger().error(
                        f'[{i}] failed to set harvester tree_id={tree_id}')
                    failed_picks += 1
                    continue
                if not self._trigger_harvest():
                    self.get_logger().warn(
                        f'[{i}] harvest of {tree_id} failed (continuing)')
                    failed_picks += 1
                    continue
                completed += 1

            self._stop()
            elapsed = time.time() - t_total
            summary = (f'Sequence done: {completed}/{len(sequence)} clusters '
                       f'(drive_fail={failed_drives}, pick_fail={failed_picks}) '
                       f'in {elapsed:.1f}s')
            self._publish_status(summary)
            response.success = (completed > 0)
            response.message = summary

        except Exception as e:
            self.get_logger().error(f'Orchestrator crashed: {e}')
            self._stop()
            response.success = False
            response.message = f'Crash: {e}'
        finally:
            self._busy = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = HarvestOrchestrator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
