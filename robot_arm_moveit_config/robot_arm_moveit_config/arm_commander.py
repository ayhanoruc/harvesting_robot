#!/usr/bin/env python3
"""
Arm Commander Node - Cartesian Goals with MoveIt 2

Moves the robot TCP to specified Cartesian positions using MoveIt IK.

Services:
    /go_to_pose (std_srvs/SetBool) - Move to current target position

Parameters:
    target_x, target_y, target_z - Target position in world frame

Usage:
    ros2 run robot_arm_moveit_config arm_commander.py

    # Move to plant 1
    ros2 param set /arm_commander target_x 0.75
    ros2 param set /arm_commander target_y -0.45
    ros2 param set /arm_commander target_z 0.42
    ros2 service call /go_to_pose std_srvs/srv/SetBool "{data: true}"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import math


class ArmCommander(Node):
    """Arm commander using Cartesian goals with MoveIt IK."""

    def __init__(self):
        super().__init__('arm_commander')

        self.callback_group = ReentrantCallbackGroup()

        # Parameters for target position
        self.declare_parameter('target_x', 0.75)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.45)

        # Named targets with pre-computed joint angles (IK solved manually)
        # Format: {'x': ..., 'y': ..., 'z': ..., 'joints': [hip, shoulder, elbow, wrist]}
        self.named_targets = {
            'plant_1': {'x': 0.75, 'y': -0.45, 'z': 0.42, 'joints': [-0.55, -0.5, 0.7, 0.0]},
            'plant_2': {'x': 0.85, 'y': 0.0, 'z': 0.52, 'joints': [0.0, -0.4, 0.6, 0.0]},
            'plant_3': {'x': 0.75, 'y': 0.45, 'z': 0.46, 'joints': [0.55, -0.5, 0.7, 0.0]},
            'reservoir': {'x': 0.0, 'y': 0.6, 'z': 0.25, 'joints': [1.57, -0.2, 0.3, 0.0]},
            'home': {'x': 0.83, 'y': 0.0, 'z': 0.40, 'joints': [0.0, 0.0, 0.0, 0.0]},
        }
        self.declare_parameter('target_name', '')

        # Joint names for the "arm" planning group (4 joints only, no gripper)
        self.arm_joint_names = ['hip', 'shoulder', 'elbow', 'wrist']

        # MoveGroup action client
        self._action_client = ActionClient(
            self,
            MoveGroup,
            'move_action',
            callback_group=self.callback_group
        )

        # Services
        self.create_service(
            SetBool,
            'go_to_pose',
            self.go_to_pose_callback,
            callback_group=self.callback_group
        )

        self.create_service(
            SetBool,
            'go_to_named',
            self.go_to_named_callback,
            callback_group=self.callback_group
        )

        # Wait for action server
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            return
        self.get_logger().info("Connected to MoveGroup!")

        # Print info
        self.get_logger().info("=" * 60)
        self.get_logger().info("ARM COMMANDER READY (Cartesian Mode)")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Named targets:")
        for name, pos in self.named_targets.items():
            self.get_logger().info(f"  {name}: ({pos['x']}, {pos['y']}, {pos['z']})")
        self.get_logger().info("-" * 60)
        self.get_logger().info("Usage (custom position):")
        self.get_logger().info("  ros2 param set /arm_commander target_x 0.75")
        self.get_logger().info("  ros2 param set /arm_commander target_y -0.45")
        self.get_logger().info("  ros2 param set /arm_commander target_z 0.42")
        self.get_logger().info("  ros2 service call /go_to_pose std_srvs/srv/SetBool \"{data: true}\"")
        self.get_logger().info("-" * 60)
        self.get_logger().info("Usage (named target):")
        self.get_logger().info("  ros2 param set /arm_commander target_name plant_1")
        self.get_logger().info("  ros2 service call /go_to_named std_srvs/srv/SetBool \"{data: true}\"")
        self.get_logger().info("=" * 60)

    def go_to_pose_callback(self, request, response):
        """Service callback for custom position."""
        if not request.data:
            response.success = False
            response.message = "Set data=true to trigger"
            return response

        x = self.get_parameter('target_x').value
        y = self.get_parameter('target_y').value
        z = self.get_parameter('target_z').value

        self.get_logger().info(f"Moving to: ({x}, {y}, {z})")

        success = self.send_pose_goal(x, y, z)

        response.success = success
        response.message = f"{'Success' if success else 'Failed'}: ({x}, {y}, {z})"
        return response

    def go_to_named_callback(self, request, response):
        """Service callback for named targets - uses joint-space goals."""
        if not request.data:
            response.success = False
            response.message = "Set data=true to trigger"
            return response

        target_name = self.get_parameter('target_name').value

        if target_name not in self.named_targets:
            response.success = False
            response.message = f"Unknown target: {target_name}. Available: {list(self.named_targets.keys())}"
            return response

        target = self.named_targets[target_name]
        joints = target['joints']
        self.get_logger().info(f"Moving to {target_name}")
        self.get_logger().info(f"  Position: ({target['x']}, {target['y']}, {target['z']})")
        self.get_logger().info(f"  Joints: {joints}")

        # Use joint-space goal (reliable for 4-DOF arm)
        success = self.send_joint_goal(joints)

        response.success = success
        response.message = f"{'Success' if success else 'Failed'}: {target_name}"
        return response

    def send_joint_goal(self, joint_values):
        """Send joint-space goal to MoveGroup (reliable for any DOF)."""
        from moveit_msgs.msg import JointConstraint

        goal_msg = MoveGroup.Goal()

        # Motion plan request
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # Joint constraints (4 arm joints only)
        goal_constraints = Constraints()

        for name, value in zip(self.arm_joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(goal_constraints)

        # Planning options
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        # Send goal
        self.get_logger().info(f"Sending joint goal: {joint_values}")

        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False

        self.get_logger().info("Goal accepted, waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error("Result is None!")
            return False

        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Goal reached!")
            return True
        else:
            self.get_logger().error(f"Failed: error_code={result.result.error_code.val}")
            return False

    def send_pose_goal(self, x, y, z):
        """Send Cartesian pose goal to MoveGroup."""
        goal_msg = MoveGroup.Goal()

        # Motion plan request
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # Position constraint
        goal_constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "world"
        position_constraint.link_name = "tcp"
        position_constraint.weight = 1.0

        # Target position with tolerance
        target_point = Point()
        target_point.x = x
        target_point.y = y
        target_point.z = z

        # Bounding volume (sphere around target) - larger tolerance
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.05]  # 5cm tolerance
        bounding_volume.primitives.append(sphere)

        primitive_pose = Pose()
        primitive_pose.position = target_point
        primitive_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(primitive_pose)

        position_constraint.constraint_region = bounding_volume
        goal_constraints.position_constraints.append(position_constraint)

        # Add loose orientation constraint (allow any orientation)
        # This helps KDL find valid IK solutions for 4-DOF arm
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "world"
        orientation_constraint.link_name = "tcp"
        orientation_constraint.orientation.w = 1.0  # Identity quaternion
        orientation_constraint.absolute_x_axis_tolerance = 3.15  # ~180 degrees
        orientation_constraint.absolute_y_axis_tolerance = 3.15
        orientation_constraint.absolute_z_axis_tolerance = 3.15
        orientation_constraint.weight = 0.1  # Low weight - position is more important
        goal_constraints.orientation_constraints.append(orientation_constraint)

        goal_msg.request.goal_constraints.append(goal_constraints)

        # Planning options
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        # Send goal
        self.get_logger().info(f"Sending Cartesian goal: ({x}, {y}, {z})")

        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().error("Goal handle is None!")
            return False

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False

        self.get_logger().info("Goal accepted, waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error("Result is None!")
            return False

        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Goal reached!")
            return True
        else:
            self.get_logger().error(f"Failed: error_code={result.result.error_code.val}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ArmCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
