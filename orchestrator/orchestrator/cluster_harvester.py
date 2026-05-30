#!/usr/bin/env python3
"""
Cluster Harvester — full single-cluster pipeline at current scout pose.

Detection-driven harvesting loop:

  do {
      1. /cluster_scan/run                  ← cluster_scanner (sweep + detect + 3D)
      2. parse JSON → cluster bolls (world XYZ)
      3. match each detection to the closest ground-truth boll in
         orchard_bolls.yaml within match_radius_m  → Gazebo model IDs
      4. sort by reach distance to arm base (closest first)
      5. /simple_cluster_harvester runtime IDs + /simple_harvest/start
         ← reuses the existing robust pick subroutine (base-rotation
            reservoir heuristic + carry-during-pick + reservoir-carry)
      6. wait for completion
  } while bolls_found and iteration < max_iterations
  emit DONE

CONTINUATION DECISION
=====================
After every pick batch we re-scan. If the scan returns 0 cluster bolls,
we're done with this cluster. The max_iterations cap protects against
infinite loops if some bolls genuinely can't be picked.

ORDERING
========
Within a batch, bolls are sorted by Euclidean distance from base_0
(arm base in world frame). Closest first → arm moves the least, success
rate per pick is highest, and partially-occluded bolls become reachable
after their neighbors are removed.

SERVICE
=======
  /cluster_harvester/run  (std_srvs/Trigger)

PARAMETERS
==========
  max_iterations       : 3       — re-scan/pick loop cap (safety)
  match_radius_m       : 0.05    — proximity to map detection → YAML boll
  arm_base_frame       : 'base_0'
  world_frame          : 'world'
  boll_inventory_yaml  : ''      — defaults to robot_arm/config/orchard_bolls.yaml
  cluster_scan_timeout_s: 90.0   — wait for /cluster_scan/run
  pick_batch_timeout_s : 600.0   — wait for one /simple_harvest/start

PREREQUISITES (all must be running)
====================================
  - moveit.launch.py (arm_commander + controllers)
  - cv_boll_detector  (/yolo/detect)
  - depth_processor   (/depth_processor/pixel_to_3d)
  - cluster_scanner   (/cluster_scan/run)
  - simple_cluster_harvester (/simple_harvest/start)
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import yaml
from std_srvs.srv import Trigger
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from tf2_ros import Buffer, TransformListener

from ament_index_python.packages import get_package_share_directory


class ClusterHarvester(Node):

    def __init__(self):
        super().__init__('cluster_harvester')
        self.cb = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('max_iterations',         3)
        self.declare_parameter('match_radius_m',         0.05)
        self.declare_parameter('arm_base_frame',         'base_0')
        self.declare_parameter('world_frame',            'world')
        self.declare_parameter('boll_inventory_yaml',    '')
        self.declare_parameter('cluster_scan_timeout_s', 120.0)
        self.declare_parameter('pick_batch_timeout_s',   600.0)

        # ── Ground-truth boll inventory (for detection→ID matching) ─
        self._boll_items: List[dict] = self._load_boll_inventory()

        # ── TF (for reach-distance sort) ────────────────────────
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True)

        # ── Pubs ────────────────────────────────────────────────
        self.status_pub = self.create_publisher(
            String, '/cluster_harvester/status', 10)

        # ── Service clients ─────────────────────────────────────
        self.scan_cli = self.create_client(
            Trigger, '/cluster_scan/run', callback_group=self.cb)
        self.harvest_cli = self.create_client(
            Trigger, '/simple_harvest/start', callback_group=self.cb)
        self.harvest_setparam_cli = self.create_client(
            SetParameters,
            '/simple_cluster_harvester/set_parameters',
            callback_group=self.cb)

        # ── Service ─────────────────────────────────────────────
        self.create_service(
            Trigger, '/cluster_harvester/run', self._on_run,
            callback_group=self.cb)

        self.get_logger().info('=' * 60)
        self.get_logger().info('CLUSTER HARVESTER ready')
        self.get_logger().info(
            f'  max_iterations={self.get_parameter("max_iterations").value}, '
            f'match_radius={self.get_parameter("match_radius_m").value:.3f}m')
        self.get_logger().info(
            f'  ground-truth inventory: {len(self._boll_items)} bolls')
        self.get_logger().info(
            '  pipeline: /cluster_scan/run → match IDs → '
            'sort by reach → /simple_harvest/start → repeat')
        self.get_logger().info('  service: /cluster_harvester/run')
        self.get_logger().info('=' * 60)

    # ─── Inventory ──────────────────────────────────────────────

    def _load_boll_inventory(self) -> List[dict]:
        path = self.get_parameter('boll_inventory_yaml').value
        if not path:
            try:
                share = get_package_share_directory('robot_arm')
                path = os.path.join(share, 'config', 'orchard_bolls.yaml')
            except Exception:
                path = ''
        if not path or not os.path.isfile(path):
            self.get_logger().error(f'boll inventory yaml not found: {path}')
            return []
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            return data.get('items', []) or []
        except Exception as e:
            self.get_logger().error(f'Failed to read {path}: {e}')
            return []

    # ─── TF / reach distance ────────────────────────────────────

    def _arm_base_world_pos(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter('world_frame').value,
                self.get_parameter('arm_base_frame').value,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0))
            return (t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z)
        except Exception as e:
            self.get_logger().warn(f'TF world←base_0 failed: {e}')
            return None

    # ─── Cluster scan ───────────────────────────────────────────

    def _wait_future(self, future, timeout_s: float):
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout_s:
                return None
            time.sleep(0.05)
        return future.result()

    def _run_cluster_scan(self) -> Optional[dict]:
        """Trigger /cluster_scan/run, then read the saved JSON it points to."""
        if not self.scan_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/cluster_scan/run unavailable')
            return None
        future = self.scan_cli.call_async(Trigger.Request())
        timeout = float(self.get_parameter('cluster_scan_timeout_s').value)
        r = self._wait_future(future, timeout)
        if r is None:
            self.get_logger().error(
                f'/cluster_scan/run timeout after {timeout}s')
            return None
        # Extract JSON path from message  (format: "... json=<path>")
        msg = r.message or ''
        if 'json=' not in msg:
            self.get_logger().warn(
                f'/cluster_scan/run returned no JSON path: {msg!r}')
            return None
        json_path = msg.split('json=', 1)[1].strip()
        try:
            with open(json_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(
                f'Failed to read scan JSON {json_path}: {e}')
            return None

    # ─── Detection → YAML ID matching ──────────────────────────

    def _match_detections_to_ids(self,
                                 scan: dict) -> Tuple[List[Tuple[str, dict]],
                                                       List[dict]]:
        """For each cluster boll in `scan`, find the closest ground-truth
        YAML boll within match_radius_m. Returns (matched, unmatched).

        matched: list of (yaml_boll_id, detection_dict)
        unmatched: list of detection_dict with no YAML twin
        """
        radius = float(self.get_parameter('match_radius_m').value)
        cluster_bolls = (scan.get('cluster', {}) or {}).get('bolls', [])
        matched: List[Tuple[str, dict]] = []
        unmatched: List[dict] = []
        used_ids = set()
        for det in cluster_bolls:
            dx, dy, dz = det['xyz']
            best_id, best_d = None, float('inf')
            for b in self._boll_items:
                bid = b.get('id')
                if not bid or bid in used_ids:
                    continue
                d = math.sqrt(
                    (dx - float(b['x'])) ** 2
                    + (dy - float(b['y'])) ** 2
                    + (dz - float(b['z'])) ** 2)
                if d < best_d:
                    best_d, best_id = d, bid
            if best_id is not None and best_d <= radius:
                matched.append((best_id, det))
                used_ids.add(best_id)
            else:
                unmatched.append(det)
        return matched, unmatched

    # ─── Sorting ────────────────────────────────────────────────

    def _sort_by_reach(self,
                       matched: List[Tuple[str, dict]]
                       ) -> List[Tuple[str, dict]]:
        base = self._arm_base_world_pos()
        if base is None:
            self.get_logger().warn(
                'No TF for arm base; falling back to unsorted order')
            return matched
        bx, by, bz = base

        def key(item):
            _id, det = item
            x, y, z = det['xyz']
            return (x - bx) ** 2 + (y - by) ** 2 + (z - bz) ** 2

        return sorted(matched, key=key)

    # ─── simple_cluster_harvester wiring ───────────────────────

    def _push_runtime_ids(self, ids: List[str]) -> bool:
        if not self.harvest_setparam_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '/simple_cluster_harvester/set_parameters unavailable')
            return False
        req = SetParameters.Request(parameters=[Parameter(
            name='boll_ids_runtime',
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                string_array_value=list(ids)))])
        future = self.harvest_setparam_cli.call_async(req)
        r = self._wait_future(future, 5.0)
        if r is None:
            return False
        return all(rr.successful for rr in r.results)

    def _trigger_harvest(self) -> Tuple[bool, str]:
        if not self.harvest_cli.wait_for_service(timeout_sec=5.0):
            return False, '/simple_harvest/start unavailable'
        future = self.harvest_cli.call_async(Trigger.Request())
        timeout = float(self.get_parameter('pick_batch_timeout_s').value)
        r = self._wait_future(future, timeout)
        if r is None:
            return False, f'/simple_harvest/start timeout after {timeout}s'
        return bool(r.success), str(r.message)

    # ─── Status ─────────────────────────────────────────────────

    def _publish_status(self, msg: str):
        self.status_pub.publish(String(data=msg))
        self.get_logger().info(f'[CH] {msg}')

    # ─── Main entry ─────────────────────────────────────────────

    def _on_run(self, request, response):
        max_iter = int(self.get_parameter('max_iterations').value)
        t_total = time.time()

        total_picked = 0
        last_status = 'no iterations ran'

        for it in range(1, max_iter + 1):
            self._publish_status(f'iter {it}/{max_iter}: scanning cluster')
            scan = self._run_cluster_scan()
            if scan is None:
                last_status = f'iter {it}: scan failed'
                self._publish_status(last_status)
                continue

            counts = scan.get('counts', {})
            n_cluster = counts.get('cluster_bolls', 0)
            self._publish_status(
                f'iter {it}: scan returned {n_cluster} cluster bolls '
                f'(unique={counts.get("unique_bolls", 0)}, '
                f'outliers={counts.get("outliers", 0)})')

            # ── CONTINUATION DECISION ────────────────────────
            if n_cluster == 0:
                last_status = f'iter {it}: 0 cluster bolls → DONE'
                self._publish_status(last_status)
                break

            matched, unmatched = self._match_detections_to_ids(scan)
            if unmatched:
                self._publish_status(
                    f'iter {it}: {len(unmatched)} detection(s) had no YAML match '
                    f'(radius={self.get_parameter("match_radius_m").value:.3f}m) — skipped')
            if not matched:
                last_status = (
                    f'iter {it}: detections found but none matched inventory — DONE')
                self._publish_status(last_status)
                break

            ordered = self._sort_by_reach(matched)
            ordered_ids = [bid for bid, _ in ordered]
            self._publish_status(
                f'iter {it}: picking {len(ordered_ids)} bolls (closest first): '
                f'{ordered_ids}')

            if not self._push_runtime_ids(ordered_ids):
                last_status = f'iter {it}: failed to push runtime IDs'
                self._publish_status(last_status)
                continue

            ok, msg = self._trigger_harvest()
            self._publish_status(
                f'iter {it}: pick batch {"OK" if ok else "FAIL"} — {msg}')
            if ok:
                # `simple_cluster_harvester` reports e.g. "picked 4/6 (failed 2)"
                total_picked += self._extract_picked_count(msg)

        # Reset runtime IDs so future calls go back to YAML mode
        self._push_runtime_ids([])

        elapsed = time.time() - t_total
        summary = (
            f'DONE in {elapsed:.1f}s | total picked across iterations '
            f'≈ {total_picked} | last={last_status}')
        self._publish_status(summary)

        response.success = True
        response.message = summary
        return response

    @staticmethod
    def _extract_picked_count(msg: str) -> int:
        """Parse 'picked N/M' from simple_cluster_harvester result message."""
        try:
            tag = 'picked '
            if tag in msg:
                tail = msg.split(tag, 1)[1].split('/', 1)[0]
                return int(tail)
        except Exception:
            pass
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = ClusterHarvester()
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
