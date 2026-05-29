#!/usr/bin/env python3
"""
Classical-CV Cotton Boll Detector (physics-based, sim-targeted).

The bolls in the sim are spheres of known physical radius r_world=0.035m
(from generate_bolls.py). The trained YOLO model expects real-world
cotton imagery and reliably fails on the sim representation, so we build
a detector around the fact that we KNOW the boll's geometry.

PIPELINE
========
Stage 0  Build a coarse candidate mask
         depth ∈ [min_d, max_d]   ← kills sky / NaN / far things
         color: brightness ≥ X, chroma ≤ Y  ← kills saturated canopy
         morph open + close

Stage 1  findContours on the mask

Stage 2  CHEAP pre-filters (skip noise before expensive depth-stat work)
         area ≥ min_area
         circularity = 4π·A/P²  ≥ min_circ

Stage 3  PHYSICS-BASED ROBUST FILTERS (the real discriminators)

         Invariant A — predicted pixel radius equals actual:
            depth at centroid → d
            r_predicted = fx · r_world / d
            ratio = r_actual / r_predicted
            reject unless ratio ∈ [1 - radius_tol, 1 + radius_tol]

         Invariant B — depth uniformity inside silhouette:
            fill contour, sample depth pixels inside
            valid samples ≥ depth_min_samples
            std(depth) ≤ depth_std_factor · r_world
            (a sphere's surface varies by ~r_world; anything else
             — branch, ground, multi-object blob, sky leak — has
             much larger spread)

         These two filters are derived from the boll's known size, not
         tuned thresholds, so they generalize across distances, lighting,
         and viewpoints.

SERVICE INTERFACE
=================
  /yolo/detect           harvester_interfaces/srv/YoloDetect   — bolls
  /yolo/detect_clusters  harvester_interfaces/srv/YoloDetect   — merged bbox

Annotated PNG saved on every call (even when 0 detections); the binary
candidate mask is saved alongside as <prefix>_<ts>_mask.png.

USAGE
=====
  ros2 run orchestrator cv_boll_detector
  ros2 service call /yolo/detect          harvester_interfaces/srv/YoloDetect '{}'
  ros2 service call /yolo/detect_clusters harvester_interfaces/srv/YoloDetect '{}'
"""

from __future__ import annotations

import math
import os
from datetime import datetime
from typing import List, Optional

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from harvester_interfaces.msg import BoundingBox
from harvester_interfaces.srv import YoloDetect


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)


class CvBollDetector(Node):

    def __init__(self):
        super().__init__('cv_boll_detector')

        # ── Parameters ─────────────────────────────────────────
        # Shape-first pipeline: SimpleBlobDetector finds light, sphere-like
        # blobs across multiple internal thresholds; then we filter each
        # candidate by color (must be desaturated / not green) and depth.
        self.declare_parameter('camera_topic',      '/camera/color/image_raw')
        self.declare_parameter('depth_topic',       '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        # Cheap pre-filters (just to skip noise pixels before running the
        # expensive depth-stats per contour; not the real discriminators).
        self.declare_parameter('blob_min_area',        15)   # pixel-count sanity floor
        self.declare_parameter('blob_min_circularity', 0.5)  # spheres → ~1.0; allow partial occlusion

        # Color mask (used only at mask-building stage to drop saturated canopy)
        self.declare_parameter('use_color_filter',     True)
        self.declare_parameter('color_min_brightness', 90)
        self.declare_parameter('color_max_chroma',     50)

        # Depth filter (also mask stage — drops sky / NaN / far things)
        self.declare_parameter('use_depth_filter', True)
        self.declare_parameter('min_depth',        0.20)
        self.declare_parameter('max_depth',        3.0)

        # ── PHYSICS-BASED ROBUST FILTERS (per contour) ────────
        # Ground truth: boll is a sphere of radius r_world.
        #
        # Invariant A: at depth d, the pixel radius MUST be ≈ fx · r_world / d.
        #   Tolerance = ±radius_tol around 1.0 (sensor noise band, not "magic").
        # Invariant B: inside the silhouette, depth values come from one sphere
        #   surface so std(depth) ≈ r_world / √3. Anything else (branches,
        #   ground, multi-object blob, sky leak) has much larger spread.
        #   Threshold = depth_std_factor · r_world.
        self.declare_parameter('boll_world_radius_m',  0.035)
        self.declare_parameter('radius_tol',           0.5)   # ±50% around predicted
        self.declare_parameter('depth_std_factor',     3.0)   # max std = 3·r_world
        self.declare_parameter('depth_min_samples',    8)     # min valid pixels inside silhouette

        # Cluster merging + output
        self.declare_parameter('cluster_pixel_distance', 150)
        self.declare_parameter('save_images',       True)
        self.declare_parameter('output_dir',
                               '/mnt/c/Users/ayhan/harvesting_ws/yolo_output')

        out = self.get_parameter('output_dir').value
        if self.get_parameter('save_images').value:
            os.makedirs(out, exist_ok=True)
            self.get_logger().info(f'Saving annotated images to: {out}')

        # ── State ──────────────────────────────────────────────
        self.bridge = CvBridge()
        self.latest_bgr: Optional[np.ndarray] = None
        self.latest_depth: Optional[np.ndarray] = None
        self.fx: Optional[float] = None  # focal length px (for radius check)

        # ── Subscriptions ──────────────────────────────────────
        self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self._image_cb, SENSOR_QOS)
        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self._depth_cb, SENSOR_QOS)
        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._cam_info_cb, SENSOR_QOS)

        # ── Services ───────────────────────────────────────────
        self.create_service(YoloDetect, '/yolo/detect', self._detect_cb)
        self.create_service(YoloDetect, '/yolo/detect_clusters', self._detect_clusters_cb)

        self.get_logger().info('=' * 60)
        self.get_logger().info('CV BOLL DETECTOR ready (depth → color → shape)')
        self.get_logger().info(
            f'  DEPTH: ∈[{self.get_parameter("min_depth").value:.2f},'
            f'{self.get_parameter("max_depth").value:.2f}]m '
            f'(enabled={self.get_parameter("use_depth_filter").value}) '
            f'← PRIMARY: kills sky / far / NaN')
        self.get_logger().info(
            f'  COLOR: brightness≥{self.get_parameter("color_min_brightness").value}, '
            f'chroma≤{self.get_parameter("color_max_chroma").value} '
            f'(enabled={self.get_parameter("use_color_filter").value}) '
            f'← kills saturated canopy')
        self.get_logger().info(
            f'  PRE: area≥{self.get_parameter("blob_min_area").value}, '
            f'circ≥{self.get_parameter("blob_min_circularity").value} '
            f'← cheap noise pre-filter')
        self.get_logger().info(
            f'  PHYSICS-A: r_actual / (fx · {self.get_parameter("boll_world_radius_m").value}m / depth) '
            f'∈ [{1 - self.get_parameter("radius_tol").value:.2f}, '
            f'{1 + self.get_parameter("radius_tol").value:.2f}] '
            f'← ROBUST: pixel radius matches depth')
        self.get_logger().info(
            f'  PHYSICS-B: std(depth_inside_silhouette) ≤ '
            f'{self.get_parameter("depth_std_factor").value:.1f}·r_world = '
            f'{self.get_parameter("depth_std_factor").value * self.get_parameter("boll_world_radius_m").value:.3f}m '
            f'← ROBUST: sphere surface is one depth band')
        self.get_logger().info('  services: /yolo/detect, /yolo/detect_clusters')
        self.get_logger().info('=' * 60)

    # ─── Sensor callbacks ──────────────────────────────────────

    def _image_cb(self, msg: Image):
        try:
            self.latest_bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'image cb: {e}')

    def _depth_cb(self, msg: Image):
        try:
            if msg.encoding == '32FC1':
                self.latest_depth = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='passthrough')
            elif msg.encoding == '16UC1':
                d16 = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='passthrough')
                self.latest_depth = d16.astype(np.float32) / 1000.0
            else:
                self.latest_depth = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='passthrough').astype(np.float32)
        except Exception as e:
            self.get_logger().error(f'depth cb: {e}')

    def _cam_info_cb(self, msg: CameraInfo):
        # K = [fx 0 cx, 0 fy cy, 0 0 1] — flat row-major
        if self.fx is None and len(msg.k) >= 5 and msg.k[0] > 0:
            self.fx = float(msg.k[0])
            self.get_logger().info(f'camera fx={self.fx:.1f}px')

    # ─── Core detection (depth-first → color → shape) ──────────

    def _build_foreground_mask(self, bgr: np.ndarray) -> tuple:
        """Build the candidate-pixel mask with diagnostic counters.

        Pipeline:
          1. depth ∈ [min_d, max_d]  →  near-foreground mask
             (kills sky/NaN/far things — the key discriminator)
          2. color: brightness ≥ X AND chroma ≤ Y
             (kills saturated canopy; safe to be strict because sky already gone)
          3. morphological open + close (clean speckle, fill tiny gaps)

        Returns (final_mask, debug_dict) where debug_dict has per-step
        pixel counts for logging.
        """
        H, W = bgr.shape[:2]
        total_px = H * W
        debug = {'total_px': total_px}

        # ── Step 1: depth validity mask ─────────────────────────
        depth = self.latest_depth
        use_depth = bool(self.get_parameter('use_depth_filter').value)
        min_d = float(self.get_parameter('min_depth').value)
        max_d = float(self.get_parameter('max_depth').value)
        if use_depth and depth is not None and depth.shape[:2] == (H, W):
            finite = np.isfinite(depth)
            in_range = (depth >= min_d) & (depth <= max_d)
            depth_mask = (finite & in_range).astype(np.uint8) * 255
        else:
            depth_mask = np.full((H, W), 255, dtype=np.uint8)
            if use_depth and depth is None:
                self.get_logger().warn('depth filter on but no depth frame yet')
        debug['depth_pass_px'] = int((depth_mask > 0).sum())

        # ── Step 2: color mask (brightness + chroma) ───────────
        use_color = bool(self.get_parameter('use_color_filter').value)
        bright_min = int(self.get_parameter('color_min_brightness').value)
        chroma_max = int(self.get_parameter('color_max_chroma').value)
        b, g, r = cv2.split(bgr)
        bi = b.astype(np.int16); gi = g.astype(np.int16); ri = r.astype(np.int16)
        brightness = ((bi + gi + ri) // 3).astype(np.uint8)
        maxc = np.maximum(np.maximum(b, g), r)
        minc = np.minimum(np.minimum(b, g), r)
        chroma = cv2.subtract(maxc, minc)
        if use_color:
            color_mask = ((brightness >= bright_min) & (chroma <= chroma_max)).astype(np.uint8) * 255
        else:
            color_mask = np.full((H, W), 255, dtype=np.uint8)
        debug['color_pass_px'] = int((color_mask > 0).sum())

        # ── Combined: depth AND color ──────────────────────────
        combined = cv2.bitwise_and(depth_mask, color_mask)
        debug['combined_px'] = int((combined > 0).sum())

        # ── Morphological cleanup ──────────────────────────────
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        opened = cv2.morphologyEx(combined, cv2.MORPH_OPEN, k, iterations=1)
        closed = cv2.morphologyEx(opened,   cv2.MORPH_CLOSE, k, iterations=1)
        debug['morph_px'] = int((closed > 0).sum())

        return closed, debug

    def _detect_bolls(self) -> tuple:
        """Depth-first → color → shape pipeline with per-step counters.

        Why depth FIRST: sky has the same gray-ish brightness as the bolls
        (light blue with very low chroma) — color alone can't tell them
        apart. Depth can: sky is NaN/inf, bolls are 0.3–3 m.

        Returns (detections, mask, message). Message embeds the full
        per-filter counter trail so it shows in the service response too.
        """
        if self.latest_bgr is None:
            self.get_logger().warn('[DETECT] no camera frame yet')
            return [], None, 'No camera frame yet'

        bgr = self.latest_bgr
        H, W = bgr.shape[:2]

        # 1. Build candidate mask (depth ∩ color, cleaned)
        mask, dbg = self._build_foreground_mask(bgr)

        # 2. Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 3. Per-contour: cheap pre-filters, then PHYSICS invariants.
        min_area  = int(self.get_parameter('blob_min_area').value)
        min_circ  = float(self.get_parameter('blob_min_circularity').value)
        min_d     = float(self.get_parameter('min_depth').value)
        max_d     = float(self.get_parameter('max_depth').value)
        r_world   = float(self.get_parameter('boll_world_radius_m').value)
        r_tol     = float(self.get_parameter('radius_tol').value)
        std_fact  = float(self.get_parameter('depth_std_factor').value)
        min_samps = int(self.get_parameter('depth_min_samples').value)

        max_depth_std = std_fact * r_world  # invariant B threshold

        depth = self.latest_depth
        depth_ok = (depth is not None and depth.shape[:2] == (H, W))

        rej = {'area': 0, 'circ': 0, 'no_depth_at_center': 0, 'depth_range': 0,
               'no_fx': 0, 'radius_mismatch': 0, 'few_samples': 0, 'depth_spread': 0}

        detections = []
        for cnt in contours:
            # ── Cheap pre-filters ──────────────────────────────
            area = cv2.contourArea(cnt)
            if area < min_area:
                rej['area'] += 1; continue
            perim = cv2.arcLength(cnt, True)
            if perim <= 0:
                rej['circ'] += 1; continue
            circularity = 4.0 * math.pi * area / (perim * perim)
            if circularity < min_circ:
                rej['circ'] += 1; continue

            # Geometry
            (fcx, fcy), fradius = cv2.minEnclosingCircle(cnt)
            cx, cy, r_px = int(fcx), int(fcy), max(2.0, float(fradius))
            x, y, w, h = cv2.boundingRect(cnt)

            # ── PHYSICS — need valid depth data ────────────────
            if not depth_ok:
                rej['no_depth_at_center'] += 1; continue
            if not (0 <= cy < H and 0 <= cx < W):
                rej['no_depth_at_center'] += 1; continue

            d_center = float(depth[cy, cx])
            if not (math.isfinite(d_center) and min_d <= d_center <= max_d):
                rej['depth_range'] += 1; continue

            # ── Invariant A: r_predicted = fx · r_world / d ────
            if self.fx is None:
                rej['no_fx'] += 1; continue
            r_predicted = self.fx * r_world / d_center
            ratio = r_px / r_predicted
            if not (1.0 - r_tol <= ratio <= 1.0 + r_tol):
                rej['radius_mismatch'] += 1; continue

            # ── Invariant B: depth uniformity inside silhouette ─
            silhouette = np.zeros((H, W), dtype=np.uint8)
            cv2.drawContours(silhouette, [cnt], -1, 255, thickness=cv2.FILLED)
            depth_inside = depth[silhouette > 0]
            finite_mask = np.isfinite(depth_inside) & (depth_inside > 0)
            valid = depth_inside[finite_mask]
            if valid.size < min_samps:
                rej['few_samples'] += 1; continue
            d_median = float(np.median(valid))
            d_std    = float(np.std(valid))
            if d_std > max_depth_std:
                rej['depth_spread'] += 1; continue

            # Confidence: combine the physical fit qualities (1.0 = perfect)
            radius_quality = max(0.0, 1.0 - abs(ratio - 1.0) / r_tol)
            depth_quality  = max(0.0, 1.0 - d_std / max_depth_std)
            conf = float(0.5 * radius_quality + 0.3 * depth_quality + 0.2 * circularity)

            detections.append({
                'u_min': int(x), 'v_min': int(y),
                'u_max': int(x + w), 'v_max': int(y + h),
                'cx': cx, 'cy': cy,
                'area': int(area),
                'radius': int(round(r_px)),
                'r_predicted_px': float(r_predicted),
                'radius_ratio': float(ratio),
                'circularity': float(circularity),
                'depth': d_median,
                'depth_std': d_std,
                'confidence': conf,
            })

        # ── Logging ─────────────────────────────────────────────
        self.get_logger().info(
            f'[DETECT] frame {W}x{H}  mask:depth_pass={dbg["depth_pass_px"]} '
            f'color_pass={dbg["color_pass_px"]} ∩={dbg["combined_px"]} '
            f'morph={dbg["morph_px"]}  contours={len(contours)}  '
            f'rej(area={rej["area"]},circ={rej["circ"]},'
            f'no_depth={rej["no_depth_at_center"]},'
            f'd_range={rej["depth_range"]},no_fx={rej["no_fx"]},'
            f'r_mis={rej["radius_mismatch"]},'
            f'few_samps={rej["few_samples"]},'
            f'd_spread={rej["depth_spread"]}) → kept={len(detections)}')
        for i, d in enumerate(detections):
            self.get_logger().info(
                f'  #{i}: ({d["cx"]},{d["cy"]}) '
                f'r={d["radius"]}px vs predicted {d["r_predicted_px"]:.1f}px '
                f'(ratio={d["radius_ratio"]:.2f}) '
                f'depth={d["depth"]:.3f}m±{d["depth_std"]*100:.1f}cm '
                f'circ={d["circularity"]:.2f} conf={d["confidence"]:.2f}')

        msg = (f'kept {len(detections)} | contours={len(contours)} '
               f'rej(area/circ/no_depth/d_range/no_fx/r_mis/few/d_spread)='
               f'{rej["area"]}/{rej["circ"]}/{rej["no_depth_at_center"]}/'
               f'{rej["depth_range"]}/{rej["no_fx"]}/{rej["radius_mismatch"]}/'
               f'{rej["few_samples"]}/{rej["depth_spread"]}')
        return detections, mask, msg

    # ─── Cluster merging (matches real_yolo_detector behavior) ─

    def _merge_to_clusters(self, dets: List[dict]) -> List[dict]:
        if not dets:
            return []
        pix_thr = int(self.get_parameter('cluster_pixel_distance').value)
        used = [False] * len(dets)
        clusters = []
        for i, di in enumerate(dets):
            if used[i]:
                continue
            group = [i]
            used[i] = True
            # Greedy single-linkage in pixel space
            queue = [i]
            while queue:
                k = queue.pop()
                for j, dj in enumerate(dets):
                    if used[j]:
                        continue
                    if math.hypot(dets[k]['cx'] - dj['cx'],
                                  dets[k]['cy'] - dj['cy']) <= pix_thr:
                        used[j] = True
                        group.append(j)
                        queue.append(j)
            u_min = min(dets[g]['u_min'] for g in group)
            v_min = min(dets[g]['v_min'] for g in group)
            u_max = max(dets[g]['u_max'] for g in group)
            v_max = max(dets[g]['v_max'] for g in group)
            cx = (u_min + u_max) // 2
            cy = (v_min + v_max) // 2
            best_conf = max(dets[g]['confidence'] for g in group)
            clusters.append({
                'u_min': u_min, 'v_min': v_min,
                'u_max': u_max, 'v_max': v_max,
                'cx': cx, 'cy': cy,
                'area': (u_max - u_min) * (v_max - v_min),
                'confidence': best_conf,
                'boll_count': len(group),
                'members': group,
            })
        return clusters

    # ─── Service callbacks ─────────────────────────────────────

    def _detect_cb(self, request, response):
        response.detections = []
        response.success = False
        dets, mask, msg = self._detect_bolls()
        if dets is None:
            response.message = msg or 'detection failed'
            return response

        for d in dets:
            b = BoundingBox()
            b.u_min = d['u_min']
            b.v_min = d['v_min']
            b.u_max = d['u_max']
            b.v_max = d['v_max']
            b.confidence = d['confidence']
            b.label = 'cotton_boll'
            b.area = d['area']
            response.detections.append(b)

        response.success = True
        response.message = f'Detected {len(dets)} bolls'

        if bool(self.get_parameter('save_images').value):
            self._save_image(dets, mask=mask, clusters=None, prefix='cv_detect')

        self.get_logger().info(f'/yolo/detect: {len(dets)} bolls')
        return response

    def _detect_clusters_cb(self, request, response):
        response.detections = []
        response.success = False
        dets, mask, msg = self._detect_bolls()
        if dets is None:
            response.message = msg or 'detection failed'
            return response

        clusters = self._merge_to_clusters(dets)
        for i, c in enumerate(clusters):
            b = BoundingBox()
            b.u_min = c['u_min']
            b.v_min = c['v_min']
            b.u_max = c['u_max']
            b.v_max = c['v_max']
            b.confidence = c['confidence']
            b.label = f'cluster_{i}'
            b.area = c['area']
            response.detections.append(b)

        response.success = True
        response.message = (
            f'Found {len(clusters)} clusters from {len(dets)} bolls')

        if bool(self.get_parameter('save_images').value):
            self._save_image(
                dets, mask=mask, clusters=clusters, prefix='cv_clusters')

        self.get_logger().info(
            f'/yolo/detect_clusters: {len(clusters)} clusters from {len(dets)} bolls')
        return response

    # ─── Annotated image saving ────────────────────────────────

    def _save_image(self, dets: List[dict], mask: Optional[np.ndarray],
                    clusters: Optional[List[dict]], prefix: str):
        if self.latest_bgr is None:
            return
        out_dir = self.get_parameter('output_dir').value
        annotated = self.latest_bgr.copy()

        # Draw individual boll detections: thin green bbox + circle around centroid
        for d in dets:
            cv2.rectangle(annotated,
                          (d['u_min'], d['v_min']),
                          (d['u_max'], d['v_max']),
                          (0, 255, 0), 1)
            r = d.get('radius', max(2, (d['u_max'] - d['u_min']) // 2))
            cv2.circle(annotated, (d['cx'], d['cy']), r, (0, 255, 0), 2)
            cv2.circle(annotated, (d['cx'], d['cy']), 2, (0, 255, 255), -1)
            label = f"{d['confidence']:.2f}"
            if d.get('depth') is not None:
                label += f" d={d['depth']:.2f}m"
            cv2.putText(annotated, label,
                        (d['u_min'], max(8, d['v_min'] - 2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # Draw clusters (thick magenta) if provided
        if clusters:
            for i, c in enumerate(clusters):
                cv2.rectangle(annotated,
                              (c['u_min'], c['v_min']),
                              (c['u_max'], c['v_max']),
                              (255, 0, 255), 3)
                cv2.putText(annotated,
                            f"cluster_{i} ({c['boll_count']} bolls)",
                            (c['u_min'], max(12, c['v_min'] - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                cv2.circle(annotated, (c['cx'], c['cy']), 5, (0, 0, 255), -1)

        # Always-on overlay summary: count + class
        cv2.putText(annotated,
                    f'CV: {len(dets)} bolls'
                    + (f' / {len(clusters)} clusters' if clusters else ''),
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2, cv2.LINE_AA)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        path = os.path.join(out_dir, f'{prefix}_{ts}.png')
        cv2.imwrite(path, annotated)
        self.get_logger().info(f'Saved: {path}')

        # Also save the binary mask alongside for tuning
        if mask is not None:
            mpath = os.path.join(out_dir, f'{prefix}_{ts}_mask.png')
            cv2.imwrite(mpath, mask)


def main(args=None):
    rclpy.init(args=args)
    node = CvBollDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
