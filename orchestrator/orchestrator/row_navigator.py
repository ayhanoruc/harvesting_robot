#!/usr/bin/env python3
"""
Row Navigator — Layer 3 pipeline: drive between clusters + harvest each.

Iterates a fixed route of tree_ids. For each one:
  1. Look up the tree's world X from orchard_tree_positions.yaml.
  2. Drive Husky to that tree's scout pose (X = tree_x, Y = scout_y,
     yaw = scout_yaw) using a closed-loop cmd_vel P-controller with TF
     feedback (world ← husky_base_link). The controller handles the
     non-holonomic DiffDrive constraint by turning in place before
     forward-driving when misaligned with the heading.
  3. Call /cluster_harvester/run — the existing detection-driven
     scan + match + pick + re-scan + DONE pipeline.
  4. Continue to the next tree.

Service:
  /row_nav/run  (std_srvs/Trigger)

Parameters:
  route                : ['tree_000', 'tree_001', 'tree_002']
  scout_y              : 4.85       aisle Y for all scout poses
  scout_yaw            : -π/2       Husky faces -Y → tree row
  pos_tol              : 0.20       drive position tolerance (m)
  yaw_tol              : 0.10       drive yaw tolerance (rad)
  max_lin              : 0.50       max linear vel (m/s)
  max_ang              : 0.60       max angular vel (rad/s)
  drive_timeout_s      : 90.0       per-waypoint drive timeout
  harvest_timeout_s    : 1200.0     per-cluster harvest timeout
  tree_positions_yaml  : ''         defaults to robot_arm/config/orchard_tree_positions.yaml

Prerequisites running:
  husky_orchard_demo.launch.py
  moveit.launch.py
  cv_boll_detector
  depth_processor
  cluster_scanner
  simple_cluster_harvester
  cluster_harvester
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


class RowNavigator(Node):

    def __init__(self):
        super().__init__('row_navigator')
        self.cb = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('route',
                               ['tree_000', 'tree_001', 'tree_002'])
        self.declare_parameter('scout_y',           4.85)
        self.declare_parameter('scout_yaw',         -math.pi / 2.0)
        self.declare_parameter('pos_tol',           0.20)
        self.declare_parameter('yaw_tol',           0.10)
        self.declare_parameter('max_lin',           0.50)
        self.declare_parameter('max_ang',           0.60)
        self.declare_parameter('drive_timeout_s',   90.0)
        self.declare_parameter('harvest_timeout_s', 1200.0)
        self.declare_parameter('tree_positions_yaml', '')

        # ── State ───────────────────────────────────────────────
        self._busy = False
        self._tree_positions = self._load_tree_positions()

        # ── TF (for husky pose feedback) ────────────────────────
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True)

        # ── Pubs ────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(
            String, '/row_nav/status', 10)

        # ── Service clients ─────────────────────────────────────
        self.harvest_cli = self.create_client(
            Trigger, '/cluster_harvester/run', callback_group=self.cb)

        # ── Service ─────────────────────────────────────────────
        self.create_service(
            Trigger, '/row_nav/run', self._on_run,
            callback_group=self.cb)

        route = list(self.get_parameter('route').value)
        self.get_logger().info('=' * 60)
        self.get_logger().info('ROW NAVIGATOR ready')
        self.get_logger().info(f'  route: {route}')
        self.get_logger().info(
            f'  scout: y={self.get_parameter("scout_y").value:.2f}m, '
            f'yaw={math.degrees(self.get_parameter("scout_yaw").value):+.0f}°')
        self.get_logger().info(
            f'  {len(self._tree_positions)} tree positions loaded')
        for tid in route:
            p = self._tree_positions.get(tid)
            if p:
                self.get_logger().info(
                    f'    {tid}: x={p[0]:.2f}, y={p[1]:.2f}')
            else:
                self.get_logger().warn(f'    {tid}: MISSING from yaml!')
        self.get_logger().info('  service: /row_nav/run')
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
        try:
            t = self.tf_buffer.lookup_transform(
                'world', 'husky_base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            return (x, y, _yaw_from_quat(q.x, q.y, q.z, q.w))
        except Exception as e:
            self.get_logger().warn(f'TF world←husky_base_link: {e}')
            return None

    # ─── Drive controller (P-controller on cmd_vel) ────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _drive_to(self, target_x, target_y, target_yaw) -> bool:
        """Closed-loop drive to (x, y, yaw) using cmd_vel + TF feedback.

        Three-stage controller (handles DiffDrive non-holonomic constraint
        — Husky can't strafe, so it must always turn-then-drive):
          1. Far + misaligned with heading → rotate in place to face target
          2. Far + aligned                 → drive forward (small yaw P-correct)
          3. At target position            → rotate to final yaw, then stop
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
                    cmd.angular.z = _clip(2.0 * yaw_err, -max_ang, max_ang)
                    cmd.linear.x = 0.0
                else:
                    cmd.linear.x = _clip(0.8 * dist, 0.05, max_lin)
                    cmd.angular.z = _clip(
                        1.5 * yaw_err, -0.5 * max_ang, 0.5 * max_ang)
            self.cmd_pub.publish(cmd)
            time.sleep(period)

        self._stop()
        self.get_logger().error(f'drive timeout (>{timeout:.0f}s)')
        return False

    # ─── cluster_harvester trigger ─────────────────────────────

    def _wait_future(self, future, timeout_s: float):
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout_s:
                return None
            time.sleep(0.05)
        return future.result()

    def _run_cluster_harvest(self) -> Tuple[bool, str]:
        if not self.harvest_cli.wait_for_service(timeout_sec=5.0):
            return False, '/cluster_harvester/run unavailable'
        future = self.harvest_cli.call_async(Trigger.Request())
        timeout = float(self.get_parameter('harvest_timeout_s').value)
        r = self._wait_future(future, timeout)
        if r is None:
            return False, f'cluster_harvester timeout after {timeout}s'
        return bool(r.success), str(r.message)

    # ─── Status ─────────────────────────────────────────────────

    def _publish_status(self, msg: str):
        self.status_pub.publish(String(data=msg))
        self.get_logger().info(f'[RN] {msg}')

    # ─── Main entry ─────────────────────────────────────────────

    def _on_run(self, request, response):
        if self._busy:
            response.success = False
            response.message = 'Already running'
            return response

        route = list(self.get_parameter('route').value)
        scout_y = float(self.get_parameter('scout_y').value)
        scout_yaw = float(self.get_parameter('scout_yaw').value)

        self._busy = True
        t_total = time.time()
        harvested = 0
        drive_fails = 0
        harvest_fails = 0

        try:
            for i, tree_id in enumerate(route, 1):
                pos = self._tree_positions.get(tree_id)
                if pos is None:
                    self._publish_status(
                        f'[{i}/{len(route)}] unknown tree_id={tree_id} — skip')
                    continue
                tree_x, _ty = pos
                target_x, target_y = tree_x, scout_y

                # ── DRIVE ─────────────────────────────────────
                self._publish_status(
                    f'[{i}/{len(route)}] driving to {tree_id} scout '
                    f'({target_x:.2f}, {target_y:.2f}, yaw={math.degrees(scout_yaw):+.0f}°)')
                if not self._drive_to(target_x, target_y, scout_yaw):
                    self._publish_status(
                        f'[{i}/{len(route)}] drive to {tree_id} FAILED — skip harvest')
                    drive_fails += 1
                    continue

                # ── HARVEST ───────────────────────────────────
                self._publish_status(
                    f'[{i}/{len(route)}] arrived at {tree_id} → /cluster_harvester/run')
                ok, msg = self._run_cluster_harvest()
                self._publish_status(
                    f'[{i}/{len(route)}] harvest {"OK" if ok else "FAIL"} — {msg}')
                if ok:
                    harvested += 1
                else:
                    harvest_fails += 1

            self._stop()
            elapsed = time.time() - t_total
            summary = (
                f'DONE in {elapsed:.1f}s | harvested {harvested}/{len(route)} '
                f'clusters (drive_fail={drive_fails}, harvest_fail={harvest_fails})')
            self._publish_status(summary)
            response.success = (harvested > 0)
            response.message = summary

        except Exception as e:
            self.get_logger().error(f'Row nav crashed: {e}')
            self._stop()
            response.success = False
            response.message = f'Crash: {e}'
        finally:
            self._busy = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RowNavigator()
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
