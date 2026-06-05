#!/usr/bin/env python3
"""
Real YOLO Detector Node

Runs actual YOLO inference on camera images for cotton detection.

Services:
    /yolo/detect (harvester_interfaces/srv/YoloDetect)
        - Returns raw YOLO detections (classes: cotton_boll, unripe-cotton)
        - Used by cluster_scanner per scan pose; only `cotton_boll` is
          treated as a pickable target downstream.

    /yolo/detect_clusters (harvester_interfaces/srv/YoloDetect)
        - Returns merged cluster bboxes derived from grouped boll detections
        - Used for: cluster-level grouping at scout pose

Model: weights/best.pt (YOLO11n, trained on the Roboflow cotton-boll-and-
cluster v5 dataset; see src/yolo_training/ for the full training package,
dataset config, metrics, and reproducible commands).

Parameters:
    - model_path: Path to YOLO model (.pt file). Defaults to the model
      installed under share/orchestrator/models/best.pt.
    - confidence: Detection confidence threshold (default: 0.7;
      harvester_modules.launch.py overrides to 0.30 for the sim, the
      training author recommends 0.54 as a balanced baseline).
    - camera_topic: Camera image topic (default: /camera/color/image_raw)
    - cluster_pixel_distance: Max pixel distance to group bolls (default: 80)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from harvester_interfaces.msg import BoundingBox
from harvester_interfaces.srv import YoloDetect

from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory


class RealYoloDetector(Node):
    """Real YOLO detection using ultralytics with cluster merging."""

    def __init__(self):
        super().__init__('real_yolo_detector')
        self.get_logger().info('Real YOLO Detector initializing...')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence', 0.7)
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('cluster_pixel_distance', 150)
        self.declare_parameter('save_images', True)
        self.declare_parameter('output_dir', '/mnt/c/Users/ayhan/harvesting_ws/yolo_output')

        model_path = self.get_parameter('model_path').value
        self.confidence = self.get_parameter('confidence').value
        camera_topic = self.get_parameter('camera_topic').value
        self.cluster_pixel_distance = self.get_parameter('cluster_pixel_distance').value
        self.save_images = self.get_parameter('save_images').value
        self.output_dir = self.get_parameter('output_dir').value

        # Create output directory if saving images
        if self.save_images:
            os.makedirs(self.output_dir, exist_ok=True)
            self.get_logger().info(f'Saving annotated images to: {self.output_dir}')

        # Find model
        if not model_path:
            # Try multiple locations
            search_paths = [
                # Installed package share directory
                os.path.join(get_package_share_directory('orchestrator'), 'models', 'best.pt'),
                # Development: src/orchestrator/models/
                os.path.join(os.path.dirname(__file__), '..', 'models', 'best.pt'),
                # Research folder fallback
                '/mnt/c/Users/ayhan/harvesting_ws/src/docs/RESEARCH/Cotton-Tracking-YOLO/best.pt',
            ]

            for path in search_paths:
                path = os.path.normpath(path)
                if os.path.exists(path):
                    model_path = path
                    break
            else:
                self.get_logger().error(f'Model not found in any of: {search_paths}')
                raise FileNotFoundError(f'YOLO model not found')

        if not os.path.exists(model_path):
            self.get_logger().error(f'Model not found: {model_path}')
            raise FileNotFoundError(f'YOLO model not found: {model_path}')

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(f'Model loaded. Classes: {self.model.names}')

        # CV Bridge
        self.bridge = CvBridge()

        # Latest frame storage
        self.latest_frame = None
        self.frame_timestamp = None

        # QoS for camera
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            sensor_qos
        )

        # Raw detection service
        self.detect_srv = self.create_service(
            YoloDetect,
            '/yolo/detect',
            self.detect_callback
        )

        # Cluster detection service (merged bolls)
        self.detect_clusters_srv = self.create_service(
            YoloDetect,
            '/yolo/detect_clusters',
            self.detect_clusters_callback
        )

        self.get_logger().info('Real YOLO Detector ready.')
        self.get_logger().info(f'  Model: {model_path}')
        self.get_logger().info(f'  Confidence: {self.confidence}')
        self.get_logger().info(f'  Camera: {camera_topic}')
        self.get_logger().info(f'  Cluster pixel distance: {self.cluster_pixel_distance}')
        self.get_logger().info(f'  Services:')
        self.get_logger().info(f'    /yolo/detect - raw detections')
        self.get_logger().info(f'    /yolo/detect_clusters - merged cluster bboxes')

    def image_callback(self, msg: Image):
        """Store latest camera frame."""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.frame_timestamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def _run_yolo(self):
        """Run YOLO inference on latest frame. Returns (boxes, success, message)."""
        if self.latest_frame is None:
            return None, False, 'No camera frame received'

        try:
            results = self.model.predict(
                source=self.latest_frame,
                conf=self.confidence,
                verbose=False
            )
            return results[0].boxes, True, 'OK'
        except Exception as e:
            return None, False, f'YOLO inference failed: {e}'

    def _save_annotated_image(self, boxes, prefix='detect'):
        """Save annotated image with bounding boxes drawn."""
        if self.latest_frame is None:
            return

        # Copy frame and draw boxes
        annotated = self.latest_frame.copy()

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].int().tolist()
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]

            # Color: green for cotton_boll, blue for cluster
            color = (0, 255, 0) if cls_name == 'cotton_boll' else (255, 0, 0)

            # Draw box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            # Draw label
            label = f'{cls_name} {conf:.2f}'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(annotated, (x1, y1 - th - 4), (x1 + tw, y1), color, -1)
            cv2.putText(annotated, label, (x1, y1 - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Save with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        filename = f'{prefix}_{timestamp}.png'
        filepath = os.path.join(self.output_dir, filename)
        cv2.imwrite(filepath, annotated)
        self.get_logger().info(f'Saved: {filepath}')

    def _save_cluster_image(self, boll_boxes, clusters):
        """Save annotated image with boll detections and merged cluster bboxes."""
        if self.latest_frame is None:
            return

        annotated = self.latest_frame.copy()

        # Draw individual boll detections (thin green)
        for boll in boll_boxes:
            cv2.rectangle(annotated,
                          (boll['u_min'], boll['v_min']),
                          (boll['u_max'], boll['v_max']),
                          (0, 255, 0), 1)

        # Draw merged cluster bboxes (thick magenta)
        for i, c in enumerate(clusters):
            cv2.rectangle(annotated,
                          (c['u_min'], c['v_min']),
                          (c['u_max'], c['v_max']),
                          (255, 0, 255), 3)
            # Label
            label = f'cluster_{i} ({c["boll_count"]} bolls)'
            cv2.putText(annotated, label, (c['u_min'], c['v_min'] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            # Center point
            cv2.circle(annotated, (c['cx'], c['cy']), 5, (0, 0, 255), -1)

        # Save with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        filename = f'clusters_{timestamp}.png'
        filepath = os.path.join(self.output_dir, filename)
        cv2.imwrite(filepath, annotated)
        self.get_logger().info(f'Saved: {filepath}')

    def detect_callback(self, request, response):
        """
        Raw YOLO detection - returns all detections as-is.
        Used for individual boll picking.
        """
        response.detections = []
        response.success = False

        boxes, success, message = self._run_yolo()
        if not success:
            response.message = message
            self.get_logger().warn(f'/yolo/detect: {message}')
            return response

        # Convert all detections to BoundingBox messages
        for box in boxes:
            bbox = BoundingBox()
            x1, y1, x2, y2 = box.xyxy[0].int().tolist()
            bbox.u_min = x1
            bbox.v_min = y1
            bbox.u_max = x2
            bbox.v_max = y2
            bbox.confidence = float(box.conf[0])
            cls_id = int(box.cls[0])
            bbox.label = self.model.names[cls_id]
            bbox.area = (x2 - x1) * (y2 - y1)
            response.detections.append(bbox)

        response.success = True
        response.message = f'Detected {len(response.detections)} objects'

        # Save annotated image
        if self.save_images and response.detections:
            self._save_annotated_image(boxes, 'detect')

        if response.detections:
            labels = [d.label for d in response.detections]
            self.get_logger().info(f'/yolo/detect: {len(labels)} - {labels}')
        else:
            self.get_logger().info('/yolo/detect: No detections')

        return response

    def detect_clusters_callback(self, request, response):
        """
        Cluster detection - merges nearby boll detections into cluster bboxes.
        Used for panoramic scan cluster finding.
        """
        response.detections = []
        response.success = False

        boxes, success, message = self._run_yolo()
        if not success:
            response.message = message
            self.get_logger().warn(f'/yolo/detect_clusters: {message}')
            return response

        # Extract boll detections only
        boll_boxes = []
        for box in boxes:
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            if cls_name == 'cotton_boll':
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                boll_boxes.append({
                    'u_min': x1,
                    'v_min': y1,
                    'u_max': x2,
                    'v_max': y2,
                    'confidence': float(box.conf[0]),
                    'cx': (x1 + x2) // 2,
                    'cy': (y1 + y2) // 2,
                })

        if not boll_boxes:
            response.success = True
            response.message = 'No boll detections to cluster'
            self.get_logger().info('/yolo/detect_clusters: No bolls detected')
            return response

        # Merge nearby bolls into clusters
        clusters = self._merge_bolls_to_clusters(boll_boxes)

        # Convert to BoundingBox messages
        for i, cluster in enumerate(clusters):
            bbox = BoundingBox()
            bbox.u_min = cluster['u_min']
            bbox.v_min = cluster['v_min']
            bbox.u_max = cluster['u_max']
            bbox.v_max = cluster['v_max']
            bbox.confidence = cluster['confidence']
            bbox.label = f'cluster_{i}'
            bbox.area = cluster['area']
            response.detections.append(bbox)

        response.success = True
        response.message = f'Found {len(clusters)} clusters from {len(boll_boxes)} bolls'

        # Save annotated image with cluster bboxes
        if self.save_images and clusters:
            self._save_cluster_image(boll_boxes, clusters)

        self.get_logger().info(
            f'/yolo/detect_clusters: {len(clusters)} clusters from {len(boll_boxes)} bolls'
        )
        for i, c in enumerate(clusters):
            self.get_logger().info(
                f'  cluster_{i}: center=({c["cx"]}, {c["cy"]}), '
                f'bolls={c["boll_count"]}, conf={c["confidence"]:.2f}'
            )

        return response

    def _merge_bolls_to_clusters(self, boll_boxes):
        """
        Group nearby boll bboxes into cluster bboxes.

        Uses complete-linkage: a boll joins a cluster only if it's within
        pixel_distance of ALL existing members. This ensures every pair
        in a cluster is within threshold.

        Note: For more robust clustering, use the spatial_detection_pipeline
        which clusters in world-space (meters) instead of pixel-space.
        """
        if not boll_boxes:
            return []

        clusters = []
        used = set()
        pixel_dist = self.cluster_pixel_distance

        def distance(b1, b2):
            return np.sqrt((b1['cx'] - b2['cx'])**2 + (b1['cy'] - b2['cy'])**2)

        def is_close_to_all(candidate, group):
            """Check if candidate is within pixel_dist of ALL members."""
            for member in group:
                if distance(candidate, member) > pixel_dist:
                    return False
            return True

        for i, box1 in enumerate(boll_boxes):
            if i in used:
                continue

            # Start new cluster with this boll
            group = [box1]
            used.add(i)

            # Find bolls close to ALL members (complete-linkage)
            changed = True
            while changed:
                changed = False
                for j, box2 in enumerate(boll_boxes):
                    if j in used:
                        continue
                    if is_close_to_all(box2, group):
                        group.append(box2)
                        used.add(j)
                        changed = True

            # Merge group into single cluster bbox
            cluster = {
                'u_min': min(b['u_min'] for b in group),
                'v_min': min(b['v_min'] for b in group),
                'u_max': max(b['u_max'] for b in group),
                'v_max': max(b['v_max'] for b in group),
                'confidence': max(b['confidence'] for b in group),
                'boll_count': len(group),
            }
            cluster['cx'] = (cluster['u_min'] + cluster['u_max']) // 2
            cluster['cy'] = (cluster['v_min'] + cluster['v_max']) // 2
            cluster['area'] = (cluster['u_max'] - cluster['u_min']) * \
                              (cluster['v_max'] - cluster['v_min'])
            clusters.append(cluster)

        return clusters


def main(args=None):
    rclpy.init(args=args)
    node = RealYoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
