"""
Depth Processor Node

Provides a service to convert 2D pixel coordinates to 3D world coordinates
using RGB-D camera data and TF transforms.

Service:
    /depth_processor/pixel_to_3d (harvester_interfaces/srv/PixelTo3D)
    - Input: u, v (pixel coordinates)
    - Output: x, y, z (world frame position), success, message

Topics subscribed:
    /camera/depth/camera_info - Camera intrinsics
    /camera/depth/image_raw - Depth image

Uses:
    - image_geometry.PinholeCameraModel for back-projection math
    - tf2_ros for camera_optical_frame -> world transform
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point, PointStamped
from harvester_interfaces.srv import PixelTo3D

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from image_geometry import PinholeCameraModel
import numpy as np
from cv_bridge import CvBridge


class DepthProcessorNode(Node):
    """
    Converts pixel coordinates to 3D world positions using depth camera data.
    """

    def __init__(self):
        super().__init__('depth_processor')
        self.get_logger().info('Depth Processor node initializing...')

        # Parameters
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('depth_scale', 1.0)  # Gazebo depth is in meters

        # Get parameters
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_image_topic = self.get_parameter('depth_image_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.depth_scale = self.get_parameter('depth_scale').value

        # Camera model (from image_geometry)
        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False

        # K matrix intrinsics (set from camera_info, bypass buggy P matrix)
        self.fx = 277.0  # defaults
        self.fy = 277.0
        self.cx = 320.0
        self.cy = 240.0

        # Latest depth image
        self.depth_image = None
        self.depth_stamp = None

        # CV Bridge for converting ROS images
        self.bridge = CvBridge()

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscribers
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            sensor_qos
        )

        self.depth_sub = self.create_subscription(
            Image,
            depth_image_topic,
            self.depth_callback,
            sensor_qos
        )

        # Service
        self.pixel_to_3d_srv = self.create_service(
            PixelTo3D,
            '/depth_processor/pixel_to_3d',
            self.pixel_to_3d_callback
        )

        self.get_logger().info('Depth Processor ready.')
        self.get_logger().info(f'  Camera info topic: {camera_info_topic}')
        self.get_logger().info(f'  Depth image topic: {depth_image_topic}')
        self.get_logger().info(f'  Service: /depth_processor/pixel_to_3d')

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics from K matrix directly (bypass P matrix bug)."""
        self.camera_model.fromCameraInfo(msg)

        # Store K matrix values directly (Gazebo P matrix may be wrong)
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        if not self.camera_info_received:
            self.camera_info_received = True
            self.get_logger().info(
                f'Camera info received: {msg.width}x{msg.height}'
            )
            self.get_logger().info(
                f'Using K matrix: fx={self.fx:.1f}, fy={self.fy:.1f}, '
                f'cx={self.cx:.1f}, cy={self.cy:.1f}'
            )
            # Debug: show P matrix mismatch
            self.get_logger().warn(
                f'P matrix has WRONG values: cx={msg.p[2]:.1f}, cy={msg.p[6]:.1f} (ignoring)'
            )

    def depth_callback(self, msg: Image):
        """Store latest depth image."""
        try:
            # Convert to numpy array
            # Gazebo depth images are typically 32FC1 (float meters)
            if msg.encoding == '32FC1':
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            elif msg.encoding == '16UC1':
                # Some cameras use 16-bit unsigned (millimeters)
                depth_16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                self.depth_image = depth_16.astype(np.float32) / 1000.0  # Convert to meters
            else:
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            self.depth_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')

    def pixel_to_3d_callback(self, request, response):
        """
        Service callback: Convert pixel (u, v) to 3D world coordinates.

        Back-projection formula:
            X_cam = (u - cx) * depth / fx
            Y_cam = (v - cy) * depth / fy
            Z_cam = depth

        Then transform from camera_optical_frame to world.
        """
        u, v = request.u, request.v

        # Wait for camera info with timeout
        import time
        timeout = 5.0  # seconds
        start = time.time()
        while not self.camera_info_received and (time.time() - start) < timeout:
            time.sleep(0.1)

        if not self.camera_info_received:
            response.success = False
            response.message = 'Camera info not received (timeout)'
            return response

        # Wait for depth image with timeout
        start = time.time()
        while self.depth_image is None and (time.time() - start) < timeout:
            time.sleep(0.1)

        if self.depth_image is None:
            response.success = False
            response.message = 'No depth image received (timeout)'
            return response

        # Validate pixel coordinates
        height, width = self.depth_image.shape[:2]
        if u < 0 or u >= width or v < 0 or v >= height:
            response.success = False
            response.message = f'Pixel ({u}, {v}) out of bounds (image: {width}x{height})'
            return response

        # Get depth at pixel
        depth = float(self.depth_image[v, u]) * self.depth_scale

        # Check for invalid depth
        if np.isnan(depth) or np.isinf(depth) or depth <= 0:
            response.success = False
            response.message = f'Invalid depth at pixel ({u}, {v}): {depth}'
            return response

        # Back-project to 3D using K matrix directly (bypass buggy P matrix)
        # Standard pinhole model: X = (u - cx) * Z / fx, Y = (v - cy) * Z / fy
        point_cam = Point()
        point_cam.x = (u - self.cx) * depth / self.fx
        point_cam.y = (v - self.cy) * depth / self.fy
        point_cam.z = depth

        self.get_logger().info(
            f'Pixel ({u}, {v}) depth={depth:.3f}m -> Camera frame: '
            f'({point_cam.x:.3f}, {point_cam.y:.3f}, {point_cam.z:.3f})'
        )

        # Transform to world frame
        try:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.camera_frame
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point = point_cam

            # Look up transform
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                rclpy.time.Time(),  # Get latest
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Log transform for debugging
            t = transform.transform.translation
            r = transform.transform.rotation
            self.get_logger().info(
                f'TF {self.camera_frame} -> {self.world_frame}: '
                f'trans=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) '
                f'rot=({r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f})'
            )

            # Apply transform
            point_world = do_transform_point(point_stamped, transform)

            response.position = point_world.point
            response.success = True
            response.message = f'Depth: {depth:.3f}m'

            self.get_logger().info(
                f'Pixel ({u}, {v}) -> World: '
                f'({point_world.point.x:.3f}, {point_world.point.y:.3f}, {point_world.point.z:.3f})'
            )

        except tf2_ros.LookupException as e:
            response.success = False
            response.message = f'TF lookup failed: {e}'
        except tf2_ros.ExtrapolationException as e:
            response.success = False
            response.message = f'TF extrapolation failed: {e}'
        except Exception as e:
            response.success = False
            response.message = f'Transform failed: {e}'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
