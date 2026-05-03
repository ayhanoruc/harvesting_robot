re#!/usr/bin/env python3
"""
TCP Monitor Node for robot_arm (iris_arm)

Subscribes to TF and prints the TCP position in world frame.
This demonstrates forward kinematics - given joint positions,
where is the tool center point?

Usage:
    ros2 run robot_arm tcp_monitor
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TCPMonitor(Node):
    def __init__(self):
        super().__init__('tcp_monitor')

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to check TCP position
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("=" * 50)
        self.get_logger().info("TCP MONITOR - robot_arm (iris_arm)")
        self.get_logger().info("Tracking: world -> tcp")
        self.get_logger().info("=" * 50)

    def timer_callback(self):
        try:
            # Get transform from world to tcp
            transform = self.tf_buffer.lookup_transform(
                'world',
                'tcp',
                rclpy.time.Time()
            )

            pos = transform.transform.translation
            rot = transform.transform.rotation

            self.get_logger().info(
                f"TCP Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} | "
                f"Orientation: x={rot.x:.3f}, y={rot.y:.3f}, z={rot.z:.3f}, w={rot.w:.3f}"
            )

        except TransformException as ex:
            self.get_logger().warn(f"Could not get transform: {ex}")


def main(args=None):
    rclpy.init(args=args)
    node = TCPMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
