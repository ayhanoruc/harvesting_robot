#!/usr/bin/env python3
"""
Cluster Scanner — stationary arm sweep, boll consolidation, cluster bbox.

Husky stays put at its scout pose. The wrist camera is swept through a
small pan/tilt grid by directly publishing JointTrajectory commands. At
each pose:
  1. Send JointTrajectory to /arm_controller/joint_trajectory.
  2. Wait for the arm to settle.
  3. Call /yolo/detect → 2D bboxes (from cv_boll_detector).
  4. For each bbox centroid, call /depth_processor/pixel_to_3d → WORLD-frame
     3D position.
  5. Accumulate.

After the sweep:
  6. Deduplicate by 3D proximity (same boll seen from multiple poses).
  7. Cluster boundary heuristic ("gap rule"):
        - sort detections by cluster_axis (world X or Y)
        - walk the sorted list, split into groups wherever the gap to the
          next detection exceeds gap_threshold_m
        - pick the LARGEST contiguous group as our target cluster
        - the rest are likely on neighboring trees (since tree spacing
          in the orchard is ~1.5m and we set gap_threshold ≈ 1.0m)
  8. Compute the cluster's 3D bbox.
  9. Save JSON (full scan + bbox) and a 2D top-down PNG showing all
     detections plus the cluster bbox.

Service:
  /cluster_scan/run  (std_srvs/Trigger)
      Starts the scan; blocks until done. Response message contains the
      cluster summary and path to the saved JSON.

Parameters:
  pan_angles_deg        : list of joint1 offsets from SCOUT (default [-12, 0, 12])
  tilt_angles_deg       : list of joint5 offsets from SCOUT (default [-8, 0, 8])
      SCOUT = HOME with joint5 rotated +π/2 (= post-launch arm_commander pose).
      A 3×3 grid gives 9 poses: center + 4 cardinals + 4 diagonals.
  scan_settle_s         : pause after each joint move (default 1.0)
  traj_duration_s       : trajectory duration for each move (default 1.2)
  tree_spacing_m        : known orchard tree-to-tree distance (default 1.5)
  gap_threshold_m       : cluster boundary if gap > this (default 1.0)
  cluster_axis          : 'x' or 'y' — horizontal axis for the gap rule.
                          For Husky at row-0 scout pose (yaw=-π/2), trees
                          are aligned along world X, so 'x' is correct.
                          (default 'x')
  dedup_radius_m        : merge detections within this (default 0.05)
  output_dir            : where to write JSON/PNG
                          (default /mnt/c/Users/ayhan/harvesting_ws/yolo_output)

Usage:
  ros2 run orchestrator cluster_scanner
  ros2 service call /cluster_scan/run std_srvs/srv/Trigger '{}'

Prerequisites running:
  - moveit.launch.py (for arm_commander, controllers)
  - cv_boll_detector (exposes /yolo/detect)
  - depth_processor   (exposes /depth_processor/pixel_to_3d)
"""

from __future__ import annotations

import json
import math
import os
import time
from datetime import datetime
from typing import List, Optional, Tuple

import numpy as np
import cv2

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import Trigger
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as MsgDuration

from harvester_interfaces.srv import YoloDetect, PixelTo3D


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# HOME pose (same as arm_commander)
HOME_JOINTS = [0.0, -0.922, 2.4494, 0.0, -1.3, 0.0]

# SCOUT pose = HOME + joint5 rotated by +90°.
# This matches arm_commander startup, which after reaching HOME immediately
# rotates joint5 by `cluster_rotate_deg` (default 90°) so the wrist camera
# tilts forward toward the canopy instead of pointing skyward. Our scan
# pivots around this pose so we sweep across the cluster, not the sky.
SCOUT_JOINTS = list(HOME_JOINTS)
SCOUT_JOINTS[4] = HOME_JOINTS[4] + math.pi / 2.0   # -1.3 + π/2 ≈ +0.27 rad


class ClusterScanner(Node):

    def __init__(self):
        super().__init__('cluster_scanner')
        self.cb = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────
        # Pan = joint1 offset from scout (0 = looking straight at canopy).
        # Tilt = joint5 offset from scout (0 = scout default ~+0.27 rad).
        # 3 cols (pan) × 4 rows (tilt) = 12 poses. Tilt sign convention:
        #   negative tilt = joint5 decreases → toward HOME (-1.3) → looks UP
        #   positive tilt = joint5 increases → away from HOME → looks DOWN
        # Extra -16° row at the top so we also catch bolls on the upper canopy.
        self.declare_parameter('pan_angles_deg',  [-12.0, 0.0, 12.0])
        self.declare_parameter('tilt_angles_deg', [-16.0, -8.0, 0.0, 8.0])
        self.declare_parameter('scan_settle_s',   1.0)
        self.declare_parameter('traj_duration_s', 1.2)
        self.declare_parameter('tree_spacing_m',  1.5)
        self.declare_parameter('gap_threshold_m', 1.0)
        self.declare_parameter('cluster_axis',    'x')
        self.declare_parameter('dedup_radius_m',  0.05)
        self.declare_parameter('output_dir',
                               '/mnt/c/Users/ayhan/harvesting_ws/yolo_output')

        # ── Pubs ────────────────────────────────────────────────
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.status_pub = self.create_publisher(
            String, '/cluster_scan/status', 10)

        # ── Service clients ─────────────────────────────────────
        self.yolo_cli = self.create_client(
            YoloDetect, '/yolo/detect', callback_group=self.cb)
        self.depth_cli = self.create_client(
            PixelTo3D, '/depth_processor/pixel_to_3d',
            callback_group=self.cb)

        # ── Service ─────────────────────────────────────────────
        self.create_service(
            Trigger, '/cluster_scan/run', self._on_run,
            callback_group=self.cb)

        out = self.get_parameter('output_dir').value
        os.makedirs(out, exist_ok=True)

        self.get_logger().info('=' * 60)
        self.get_logger().info('CLUSTER SCANNER ready')
        self.get_logger().info(
            f'  SCOUT base: joint1={SCOUT_JOINTS[0]:+.2f}, '
            f'joint5={SCOUT_JOINTS[4]:+.2f} (= HOME + π/2 on joint5)')
        self.get_logger().info(
            f'  pan offsets: {list(self.get_parameter("pan_angles_deg").value)} deg (joint1)')
        self.get_logger().info(
            f'  tilt offsets: {list(self.get_parameter("tilt_angles_deg").value)} deg (joint5)')
        self.get_logger().info(
            f'  tree_spacing={self.get_parameter("tree_spacing_m").value}m,  '
            f'gap_threshold={self.get_parameter("gap_threshold_m").value}m  '
            f'(axis={self.get_parameter("cluster_axis").value})')
        self.get_logger().info(
            f'  dedup_radius={self.get_parameter("dedup_radius_m").value}m')
        self.get_logger().info(f'  output_dir={out}')
        self.get_logger().info('  service: /cluster_scan/run')
        self.get_logger().info('=' * 60)

    # ─── Helpers ────────────────────────────────────────────────

    def _publish_status(self, msg: str):
        self.status_pub.publish(String(data=msg))
        self.get_logger().info(f'[SCAN] {msg}')

    def _send_joint_goal(self, joints: List[float], duration_s: float):
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = list(joints)
        pt.time_from_start = MsgDuration(
            sec=int(duration_s),
            nanosec=int((duration_s % 1) * 1e9))
        traj.points = [pt]
        self.traj_pub.publish(traj)

    def _wait_future(self, future, timeout_s: float):
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout_s:
                return None
            time.sleep(0.05)
        return future.result()

    def _call_yolo_detect(self) -> list:
        if not self.yolo_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                '/yolo/detect unavailable — is cv_boll_detector running?')
            return []
        f = self.yolo_cli.call_async(YoloDetect.Request())
        r = self._wait_future(f, 15.0)
        if r is None:
            self.get_logger().warn('/yolo/detect timeout')
            return []
        return list(r.detections)

    def _pixel_to_3d_world(self, u: int, v: int):
        if not self.depth_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                '/depth_processor/pixel_to_3d unavailable — is depth_processor running?')
            return None
        req = PixelTo3D.Request()
        req.u = int(u)
        req.v = int(v)
        f = self.depth_cli.call_async(req)
        r = self._wait_future(f, 5.0)
        if r is None or not r.success:
            return None
        return (float(r.position.x), float(r.position.y), float(r.position.z))

    # ─── Consolidation ──────────────────────────────────────────

    def _dedup(self, raw: list) -> list:
        """Merge raw detections within dedup_radius_m of each other (3D).

        Returns a list of dicts with keys: xyz, n_obs, confidence,
        and per-pose membership for traceability.
        """
        rad = float(self.get_parameter('dedup_radius_m').value)
        merged: List[dict] = []
        for det in raw:
            x, y, z = det['xyz']
            absorbed = False
            for m in merged:
                mx, my, mz = m['xyz']
                if math.dist((x, y, z), (mx, my, mz)) <= rad:
                    n = m['n_obs']
                    m['xyz'] = (
                        (mx * n + x) / (n + 1),
                        (my * n + y) / (n + 1),
                        (mz * n + z) / (n + 1),
                    )
                    m['n_obs'] = n + 1
                    m['confidence'] = max(m['confidence'], det['confidence'])
                    m['poses'].append(det['pose_label'])
                    absorbed = True
                    break
            if not absorbed:
                merged.append({
                    'xyz': (x, y, z),
                    'n_obs': 1,
                    'confidence': det['confidence'],
                    'poses': [det['pose_label']],
                })
        return merged

    def _find_cluster_group(self, dets: list) -> Tuple[list, list]:
        """Gap-rule clustering on cluster_axis.

        Sort detections by their cluster-axis coordinate; walk the list
        and split where the gap between consecutive detections exceeds
        gap_threshold_m. The LARGEST resulting group is returned as the
        target cluster; the rest are returned as outliers (neighbors).
        """
        if not dets:
            return [], []
        axis = self.get_parameter('cluster_axis').value
        axis_idx = 0 if axis == 'x' else (1 if axis == 'y' else 2)
        gap_thr = float(self.get_parameter('gap_threshold_m').value)

        sorted_dets = sorted(dets, key=lambda d: d['xyz'][axis_idx])
        groups: List[list] = []
        current = [sorted_dets[0]]
        for i in range(1, len(sorted_dets)):
            prev_v = sorted_dets[i - 1]['xyz'][axis_idx]
            curr_v = sorted_dets[i]['xyz'][axis_idx]
            if abs(curr_v - prev_v) > gap_thr:
                groups.append(current)
                current = [sorted_dets[i]]
            else:
                current.append(sorted_dets[i])
        groups.append(current)

        # Largest contiguous group → target cluster
        target = max(groups, key=lambda g: len(g))
        outliers = [d for g in groups if g is not target for d in g]
        self.get_logger().info(
            f'  gap-rule: {len(groups)} groups along {axis} '
            f'→ target has {len(target)} bolls, '
            f'{len(outliers)} outliers (likely neighbors)')
        for i, g in enumerate(groups):
            vals = [d['xyz'][axis_idx] for d in g]
            tag = ' ← TARGET' if g is target else ''
            self.get_logger().info(
                f'    group[{i}]: n={len(g)} '
                f'{axis}∈[{min(vals):.3f}, {max(vals):.3f}]{tag}')
        return target, outliers

    def _bbox_of(self, dets: list) -> dict:
        if not dets:
            return {}
        xs = [d['xyz'][0] for d in dets]
        ys = [d['xyz'][1] for d in dets]
        zs = [d['xyz'][2] for d in dets]
        return {
            'min': [min(xs), min(ys), min(zs)],
            'max': [max(xs), max(ys), max(zs)],
            'center': [
                (min(xs) + max(xs)) / 2.0,
                (min(ys) + max(ys)) / 2.0,
                (min(zs) + max(zs)) / 2.0,
            ],
            'extent': [
                max(xs) - min(xs),
                max(ys) - min(ys),
                max(zs) - min(zs),
            ],
        }

    # ─── Top-down PNG visualization ────────────────────────────

    def _save_topdown_png(self, target: list, outliers: list,
                          bbox: dict, ts: str):
        """Render a top-down (world XY) plot of detections + cluster bbox."""
        out_dir = self.get_parameter('output_dir').value
        W, H = 800, 800
        margin = 60
        canvas = np.full((H, W, 3), 245, dtype=np.uint8)

        all_pts = [(d['xyz'][0], d['xyz'][1]) for d in target + outliers]
        if not all_pts:
            return
        xs = [p[0] for p in all_pts]
        ys = [p[1] for p in all_pts]
        # Pad with bbox so it fits
        if bbox:
            xs += [bbox['min'][0], bbox['max'][0]]
            ys += [bbox['min'][1], bbox['max'][1]]
        x_min, x_max = min(xs) - 0.1, max(xs) + 0.1
        y_min, y_max = min(ys) - 0.1, max(ys) + 0.1
        sx = (W - 2 * margin) / max(1e-6, x_max - x_min)
        sy = (H - 2 * margin) / max(1e-6, y_max - y_min)
        s = min(sx, sy)

        def w2p(wx, wy):
            # World X → screen X (right), world Y → screen Y (inverted, up)
            px = int(margin + (wx - x_min) * s)
            py = int(H - margin - (wy - y_min) * s)
            return (px, py)

        # Axis grid
        cv2.line(canvas, (margin, margin), (margin, H - margin), (210, 210, 210), 1)
        cv2.line(canvas, (margin, H - margin), (W - margin, H - margin), (210, 210, 210), 1)
        cv2.putText(canvas, 'X →', (W - margin - 50, H - margin + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (110, 110, 110), 1)
        cv2.putText(canvas, 'Y ↑', (margin - 35, margin + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (110, 110, 110), 1)

        # Cluster bbox (magenta rectangle)
        if bbox:
            p1 = w2p(bbox['min'][0], bbox['min'][1])
            p2 = w2p(bbox['max'][0], bbox['max'][1])
            cv2.rectangle(canvas,
                          (min(p1[0], p2[0]), min(p1[1], p2[1])),
                          (max(p1[0], p2[0]), max(p1[1], p2[1])),
                          (200, 30, 200), 2)
            c = bbox['center']
            cp = w2p(c[0], c[1])
            cv2.drawMarker(canvas, cp, (200, 30, 200),
                           markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)
            cv2.putText(canvas,
                        f'cluster ({c[0]:.2f},{c[1]:.2f},{c[2]:.2f})',
                        (cp[0] + 8, cp[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 30, 200), 1)

        # Outliers (gray dots) and target (green dots)
        for d in outliers:
            p = w2p(d['xyz'][0], d['xyz'][1])
            cv2.circle(canvas, p, 5, (140, 140, 140), -1)
        for d in target:
            p = w2p(d['xyz'][0], d['xyz'][1])
            cv2.circle(canvas, p, 6, (40, 180, 40), -1)
            cv2.circle(canvas, p, 6, (20, 100, 20), 1)

        # Title
        cv2.putText(canvas,
                    f'cluster_scan {ts}: target={len(target)} bolls, '
                    f'outliers={len(outliers)}',
                    (margin, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 40), 1)

        path = os.path.join(out_dir, f'cluster_topdown_{ts}.png')
        cv2.imwrite(path, canvas)
        self.get_logger().info(f'  saved top-down map: {path}')

    # ─── Main entry ─────────────────────────────────────────────

    def _on_run(self, request, response):
        pan_angles = list(self.get_parameter('pan_angles_deg').value)
        tilt_angles = list(self.get_parameter('tilt_angles_deg').value)
        settle = float(self.get_parameter('scan_settle_s').value)
        traj_t = float(self.get_parameter('traj_duration_s').value)

        total_poses = len(pan_angles) * len(tilt_angles)
        self._publish_status(
            f'Start. {total_poses} poses (pan×tilt = '
            f'{len(pan_angles)}×{len(tilt_angles)})')

        all_raw: List[dict] = []
        t_total = time.time()

        # Go to SCOUT pose first (HOME + joint5+90° — same pose arm_commander
        # ends up in after launch). This is the natural "looking at canopy"
        # pose; we pivot around it.
        self._send_joint_goal(SCOUT_JOINTS, traj_t)
        time.sleep(traj_t + 0.3)

        # Sweep (rows = tilt, cols = pan — like a TV raster).
        # joint1 pan and joint5 tilt are offsets FROM scout.
        pose_idx = 0
        for tilt_deg in tilt_angles:
            for pan_deg in pan_angles:
                pose_idx += 1
                pose_label = f'p{pan_deg:+.1f}_t{tilt_deg:+.1f}'

                joints = list(SCOUT_JOINTS)
                joints[0] = SCOUT_JOINTS[0] + math.radians(pan_deg)   # joint1 pan
                joints[4] = SCOUT_JOINTS[4] + math.radians(tilt_deg)  # joint5 tilt

                self._publish_status(
                    f'pose {pose_idx}/{total_poses}: pan={pan_deg:+.1f}° tilt={tilt_deg:+.1f}°')
                self._send_joint_goal(joints, traj_t)
                time.sleep(traj_t + settle)

                # Detect bolls in this view
                bboxes = self._call_yolo_detect()
                self._publish_status(f'  /yolo/detect → {len(bboxes)} bolls')

                pose_world: List[Tuple[float, float, float]] = []
                for bb in bboxes:
                    cu = (bb.u_min + bb.u_max) // 2
                    cv = (bb.v_min + bb.v_max) // 2
                    xyz = self._pixel_to_3d_world(cu, cv)
                    if xyz is None:
                        continue
                    pose_world.append(xyz)
                    all_raw.append({
                        'xyz': xyz,
                        'confidence': float(bb.confidence),
                        'pose_idx': pose_idx,
                        'pose_label': pose_label,
                        'pixel': (int(cu), int(cv)),
                    })
                if pose_world:
                    for xyz in pose_world:
                        self.get_logger().info(
                            f'    boll @ world ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})')

        # Return arm to SCOUT (centered camera view, matches arm_commander startup)
        self._send_joint_goal(SCOUT_JOINTS, traj_t)
        time.sleep(traj_t)

        # Consolidate
        self._publish_status(
            f'sweep done in {time.time() - t_total:.1f}s. '
            f'raw={len(all_raw)}; deduplicating @ '
            f'{self.get_parameter("dedup_radius_m").value:.3f}m radius')
        unique = self._dedup(all_raw)
        self._publish_status(f'unique bolls after dedup: {len(unique)}')

        target, outliers = self._find_cluster_group(unique)
        target_bbox = self._bbox_of(target)

        if target_bbox:
            c, ex = target_bbox['center'], target_bbox['extent']
            self._publish_status(
                f'CLUSTER: {len(target)} bolls   '
                f'center=({c[0]:.2f},{c[1]:.2f},{c[2]:.2f})   '
                f'extent=({ex[0]:.2f},{ex[1]:.2f},{ex[2]:.2f})   '
                f'rejected_neighbors={len(outliers)}')
        else:
            self._publish_status('CLUSTER: no detections')

        # Save artifacts
        out_dir = self.get_parameter('output_dir').value
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        data = {
            'timestamp': ts,
            'elapsed_s': round(time.time() - t_total, 2),
            'params': {
                'pan_angles_deg': pan_angles,
                'tilt_angles_deg': tilt_angles,
                'tree_spacing_m': float(self.get_parameter('tree_spacing_m').value),
                'gap_threshold_m': float(self.get_parameter('gap_threshold_m').value),
                'cluster_axis': self.get_parameter('cluster_axis').value,
                'dedup_radius_m': float(self.get_parameter('dedup_radius_m').value),
            },
            'counts': {
                'poses_swept': pose_idx,
                'raw_detections': len(all_raw),
                'unique_bolls': len(unique),
                'cluster_bolls': len(target),
                'outliers': len(outliers),
            },
            'cluster': {
                'bbox': target_bbox,
                'bolls': [
                    {'xyz': list(d['xyz']),
                     'n_obs': d['n_obs'],
                     'confidence': d['confidence'],
                     'poses': d['poses']}
                    for d in target
                ],
            },
            'outliers': [
                {'xyz': list(d['xyz']),
                 'n_obs': d['n_obs'],
                 'confidence': d['confidence'],
                 'poses': d['poses']}
                for d in outliers
            ],
            'raw_detections_full': [
                {'xyz': list(d['xyz']),
                 'confidence': d['confidence'],
                 'pose_label': d['pose_label'],
                 'pixel': list(d['pixel'])}
                for d in all_raw
            ],
        }
        json_path = os.path.join(out_dir, f'cluster_scan_{ts}.json')
        with open(json_path, 'w') as f:
            json.dump(data, f, indent=2, default=str)
        self.get_logger().info(f'  saved JSON: {json_path}')

        self._save_topdown_png(target, outliers, target_bbox, ts)

        # Build response
        if target_bbox:
            c, ex = target_bbox['center'], target_bbox['extent']
            msg = (
                f'OK: {len(target)} cluster bolls   '
                f'center=({c[0]:.2f},{c[1]:.2f},{c[2]:.2f})   '
                f'extent=({ex[0]:.2f},{ex[1]:.2f},{ex[2]:.2f})   '
                f'(raw={len(all_raw)}, unique={len(unique)}, '
                f'outliers={len(outliers)})   json={json_path}'
            )
            response.success = True
        else:
            msg = (f'EMPTY: no detections after {pose_idx} poses; '
                   f'see {json_path} for details')
            response.success = False
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ClusterScanner()
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
