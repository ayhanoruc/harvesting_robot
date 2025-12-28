#!/usr/bin/env python3
"""
ROS2 Arm Control Node

Interactive command-line interface to control the 4-DOF robot arm.

Commands:
  move    - Set all joint positions
  stop    - Return to initial position and exit
  release - Open gripper
  close   - Close gripper
  change  - Change single joint position

Usage:
  ros2 run robot_arm write_pos.py
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller_node')

        # Publisher for arm commands
        self.pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # Joint name mapping
        self.joint_dict = {'hip': 0, 'shoulder': 1, 'elbow': 2, 'wrist': 3}
        self.joint_names = ['hip', 'shoulder', 'elbow', 'wrist', 'l_g_base', 'r_g_base']

        # Initialize trajectory message
        self.jt = JointTrajectory()
        self.jt.header.frame_id = "base_link"
        self.jt.joint_names = self.joint_names

        # Initial positions
        self.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper_open = True

        self.get_logger().info("=" * 50)
        self.get_logger().info("4-DOF ROBOT ARM CONTROLLER (ROS2)")
        self.get_logger().info("=" * 50)
        self.get_logger().info("Commands:")
        self.get_logger().info("  move    - Set hip, shoulder, elbow, wrist positions")
        self.get_logger().info("  stop    - Return to home and exit")
        self.get_logger().info("  release - Open gripper")
        self.get_logger().info("  close   - Close gripper")
        self.get_logger().info("  change  - Change single joint (format: joint_name : value)")
        self.get_logger().info("=" * 50)

    def send_trajectory(self):
        """Send current positions as trajectory command."""
        self.jt.header.stamp = self.get_clock().now().to_msg()

        point = JointTrajectoryPoint()
        point.positions = self.positions.copy()
        point.time_from_start = Duration(sec=1, nanosec=0)

        self.jt.points = [point]
        self.pub.publish(self.jt)
        self.get_logger().info(f"Sent positions: {self.positions}")

    def run(self):
        """Main control loop."""
        try:
            while rclpy.ok():
                command = input("\nEnter command: ").strip().lower()

                if command == "stop":
                    self.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    self.send_trajectory()
                    self.get_logger().info("Returning to home. Goodbye!")
                    break

                elif command == "move":
                    for i in range(4):
                        try:
                            pos = float(input(f"  {self.joint_names[i]} position: "))
                            self.positions[i] = pos
                        except ValueError:
                            self.get_logger().warn("Invalid input, keeping previous value")
                    self.send_trajectory()

                elif command == "release":
                    if self.gripper_open:
                        self.get_logger().info("Gripper already open!")
                    else:
                        self.positions[4] = 0.0   # l_g_base
                        self.positions[5] = 0.0   # r_g_base
                        self.gripper_open = True
                        self.send_trajectory()
                        self.get_logger().info("Gripper released")

                elif command == "close":
                    if not self.gripper_open:
                        self.get_logger().info("Gripper already closed!")
                    else:
                        self.positions[4] = 0.5236   # ~30 degrees
                        self.positions[5] = -0.5236
                        self.gripper_open = False
                        self.send_trajectory()
                        self.get_logger().info("Gripper closed")

                elif command == "change":
                    try:
                        line = input("  joint_name : value >> ").strip()
                        parts = line.replace(":", " ").split()
                        if len(parts) >= 2:
                            joint_name = parts[0].lower()
                            value = float(parts[-1])
                            if joint_name in self.joint_dict:
                                idx = self.joint_dict[joint_name]
                                self.positions[idx] = value
                                self.send_trajectory()
                                self.get_logger().info(f"Set {joint_name} to {value}")
                            else:
                                self.get_logger().warn(f"Unknown joint: {joint_name}")
                                self.get_logger().info(f"Valid joints: {list(self.joint_dict.keys())}")
                        else:
                            self.get_logger().warn("Format: joint_name : value")
                    except ValueError:
                        self.get_logger().warn("Invalid value")

                else:
                    self.get_logger().warn(f"Unknown command: '{command}'")
                    self.get_logger().info("Valid commands: move, stop, release, close, change")

        except KeyboardInterrupt:
            self.get_logger().info("\nInterrupted by user")


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
