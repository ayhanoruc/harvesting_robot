"""
TCP Monitor Node - Forward Kinematics Demo

This node demonstrates Forward Kinematics by:
1. Listening to the TF tree
2. Looking up the transform from 'world' to 'tcp'
3. Printing the TCP position in world coordinates

This answers: "Given current joint positions, where is the TCP (cursor)?"

Run with:
    ros2 run example_arm_description tcp_monitor
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math


def quaternion_to_euler(q):
    """Convert quaternion (x, y, z, w) to euler angles (roll, pitch, yaw) in degrees."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


class TcpMonitor(Node):
    """Node that monitors and prints TCP position using TF lookups."""

    def __init__(self):
        super().__init__('tcp_monitor')

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameters
        self.declare_parameter('source_frame', 'world')
        self.declare_parameter('target_frame', 'tcp')
        self.declare_parameter('update_rate', 2.0)  # Hz

        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        update_rate = self.get_parameter('update_rate').value

        # Timer to periodically lookup transform
        self.timer = self.create_timer(1.0 / update_rate, self.timer_callback)

        self.get_logger().info(
            f'TCP Monitor started: looking up {self.source_frame} -> {self.target_frame}'
        )
        self.get_logger().info('Move the joint sliders and watch the TCP position change!')
        self.get_logger().info('-' * 60)

    def timer_callback(self):
        """Look up the transform and print TCP position."""
        try:
            # Look up transform from source_frame to target_frame
            # This is the core of Forward Kinematics in ROS!
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time()  # Get latest available
            )

            # Extract position
            pos = transform.transform.translation
            rot = transform.transform.rotation

            # Convert quaternion to euler for readability
            roll, pitch, yaw = quaternion_to_euler(rot)

            # Print in a nice format
            self.get_logger().info(
                f'TCP in {self.source_frame}: '
                f'pos=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) '
                f'rpy=({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)'
            )

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not get transform {self.source_frame} -> {self.target_frame}: {ex}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TcpMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
