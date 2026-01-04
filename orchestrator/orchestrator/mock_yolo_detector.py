#!/usr/bin/env python3
"""
Mock YOLO Detector Node

Simulates YOLO cotton detection by projecting known cluster positions
to pixel coordinates. Used for testing the spatial detection pipeline
before integrating real YOLO inference.

Service:
    /yolo/detect (harvester_interfaces/srv/YoloDetect)
        - Returns bounding boxes for visible cotton clusters

How it works:
    1. Load known cluster positions from environment_config.yaml
    2. Get camera pose via TF (camera_optical_frame -> world)
    3. Project 3D cluster positions to 2D pixels using camera intrinsics
    4. If pixel is within image bounds, return a bounding box
    5. Add slight randomness to simulate real detection variation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import CameraInfo
from harvester_interfaces.msg import BoundingBox
from harvester_interfaces.srv import YoloDetect

import tf2_ros
import numpy as np
import yaml
import os
import random

from ament_index_python.packages import get_package_share_directory


class MockYoloDetector(Node):
    """Simulates YOLO detection by projecting known positions."""

    def __init__(self):
        super().__init__('mock_yolo_detector')
        self.get_logger().info('Mock YOLO Detector initializing...')

        # Parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('bbox_size', 60)  # Approximate box size in pixels
        self.declare_parameter('bbox_variation', 15)  # Random size variation
        self.declare_parameter('confidence_base', 0.85)  # Base confidence
        self.declare_parameter('confidence_variation', 0.1)  # Random confidence variation
        self.declare_parameter('detection_noise_pixels', 5)  # Pixel noise in center

        self.camera_frame = self.get_parameter('camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.bbox_size = self.get_parameter('bbox_size').value
        self.bbox_var = self.get_parameter('bbox_variation').value
        self.conf_base = self.get_parameter('confidence_base').value
        self.conf_var = self.get_parameter('confidence_variation').value
        self.noise_px = self.get_parameter('detection_noise_pixels').value

        # Load cluster positions from config
        self.clusters = self._load_clusters()
        self.get_logger().info(f'Loaded {len(self.clusters)} cluster positions')

        # Camera intrinsics
        self.camera_matrix = None
        self.image_width = 640
        self.image_height = 480

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS for camera info
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscribe to camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            sensor_qos
        )

        # Detection service
        self.detect_srv = self.create_service(
            YoloDetect,
            '/yolo/detect',
            self.detect_callback
        )

        self.get_logger().info('Mock YOLO Detector ready.')
        self.get_logger().info('  Service: /yolo/detect')

    def _load_clusters(self) -> dict:
        """Load cluster positions from environment_config.yaml."""
        config_file = self.get_parameter('config_file').value

        if not config_file:
            # Try to find via ROS2 package share directory
            try:
                pkg_share = get_package_share_directory('robot_arm')
                config_file = os.path.join(pkg_share, 'config', 'environment_config.yaml')
            except Exception:
                # Fallback to relative path (for development)
                config_file = os.path.join(
                    os.path.dirname(__file__),
                    '..', '..', '..', 'robot_arm', 'config', 'environment_config.yaml'
                )
                config_file = os.path.normpath(config_file)

        clusters = {}

        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            if 'clusters' in config:
                for name, data in config['clusters'].items():
                    pos = data.get('position', [0, 0, 0])
                    clusters[name] = {
                        'position': np.array(pos),
                        'description': data.get('description', '')
                    }
                    self.get_logger().info(
                        f'  {name}: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]'
                    )
        except Exception as e:
            self.get_logger().warn(f'Failed to load config: {e}')
            # Fallback: hardcoded positions
            clusters = {
                'cluster_1': {'position': np.array([0.75, 0.45, 0.46])},
                'cluster_2': {'position': np.array([0.85, 0.0, 0.52])},
                'cluster_3': {'position': np.array([0.75, -0.45, 0.42])},
            }
            self.get_logger().info('Using fallback cluster positions')

        return clusters

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics."""
        if self.camera_matrix is None:
            # K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.image_width = msg.width
            self.image_height = msg.height
            self.get_logger().info(
                f'Camera info received: {msg.width}x{msg.height}, '
                f'fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}'
            )

    def detect_callback(self, request, response):
        """
        Detect cotton clusters visible in current camera view.
        Projects known 3D positions to 2D and returns bounding boxes.
        """
        response.detections = []
        response.success = False

        # Wait for camera info with timeout
        import time
        timeout = 5.0
        start = time.time()
        while self.camera_matrix is None and (time.time() - start) < timeout:
            time.sleep(0.1)

        if self.camera_matrix is None:
            response.message = 'Camera info not received (timeout)'
            return response

        # Get camera transform
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.world_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except tf2_ros.LookupException as e:
            response.message = f'TF lookup failed: {e}'
            return response
        except tf2_ros.ExtrapolationException as e:
            response.message = f'TF extrapolation failed: {e}'
            return response

        # Extract rotation and translation from transform
        t = transform.transform.translation
        q = transform.transform.rotation

        # Convert quaternion to rotation matrix
        R = self._quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])

        # Project each cluster
        for name, data in self.clusters.items():
            pos_world = data['position']

            # Transform to camera frame: P_cam = R * P_world + T
            pos_cam = R @ pos_world + T

            # Check if in front of camera (Z > 0 in camera frame)
            if pos_cam[2] <= 0.1:  # Too close or behind
                continue

            # Project to pixel coordinates
            # p = K * [X/Z, Y/Z, 1]
            px = self.camera_matrix[0, 0] * pos_cam[0] / pos_cam[2] + self.camera_matrix[0, 2]
            py = self.camera_matrix[1, 1] * pos_cam[1] / pos_cam[2] + self.camera_matrix[1, 2]

            # Add detection noise
            px += random.uniform(-self.noise_px, self.noise_px)
            py += random.uniform(-self.noise_px, self.noise_px)

            # Check if within image bounds (with margin for bbox)
            margin = self.bbox_size // 2 + self.bbox_var
            if px < margin or px >= self.image_width - margin:
                continue
            if py < margin or py >= self.image_height - margin:
                continue

            # Create bounding box with random variation
            half_w = (self.bbox_size + random.randint(-self.bbox_var, self.bbox_var)) // 2
            half_h = (self.bbox_size + random.randint(-self.bbox_var, self.bbox_var)) // 2

            # Distance affects apparent size (closer = bigger box)
            distance_factor = 1.0 / max(pos_cam[2], 0.3)  # Inverse distance
            half_w = int(half_w * distance_factor * 0.5)
            half_h = int(half_h * distance_factor * 0.5)

            # Clamp to reasonable range
            half_w = max(20, min(150, half_w))
            half_h = max(20, min(150, half_h))

            bbox = BoundingBox()
            bbox.u_min = int(px - half_w)
            bbox.v_min = int(py - half_h)
            bbox.u_max = int(px + half_w)
            bbox.v_max = int(py + half_h)
            bbox.confidence = min(1.0, max(0.0,
                self.conf_base + random.uniform(-self.conf_var, self.conf_var)
            ))
            bbox.label = name  # Use cluster name as label
            bbox.area = (bbox.u_max - bbox.u_min) * (bbox.v_max - bbox.v_min)

            response.detections.append(bbox)

            self.get_logger().debug(
                f'Detected {name} at pixel ({px:.0f}, {py:.0f}), '
                f'distance={pos_cam[2]:.2f}m, area={bbox.area}'
            )

        response.success = True
        response.message = f'Detected {len(response.detections)} clusters'

        if response.detections:
            self.get_logger().info(
                f'YOLO detect: {len(response.detections)} detections - '
                f'{[d.label for d in response.detections]}'
            )
        else:
            self.get_logger().info('YOLO detect: No clusters in view')

        return response

    def _quaternion_to_rotation_matrix(self, x, y, z, w):
        """Convert quaternion to 3x3 rotation matrix."""
        # Normalize
        n = np.sqrt(x*x + y*y + z*z + w*w)
        x, y, z, w = x/n, y/n, z/n, w/n

        return np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])


def main(args=None):
    rclpy.init(args=args)
    node = MockYoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
