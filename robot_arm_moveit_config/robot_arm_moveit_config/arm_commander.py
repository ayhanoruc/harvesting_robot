#!/usr/bin/env python3
"""
Arm Commander Node - Cartesian Goals with MoveIt 2

Moves the robot TCP to specified Cartesian positions using MoveIt IK.
Loads targets from environment_config.yaml (single source of truth).

Services:
    /go_to_pose (std_srvs/SetBool) - Move to target_x/y/z position
    /go_to_named (std_srvs/SetBool) - Move to named target from config

Parameters:
    config_file - Path to environment_config.yaml
    target_x, target_y, target_z - Custom target position
    target_name - Named target (cluster_1, reservoir, explore_start, etc.)

Usage:
    ros2 run robot_arm_moveit_config arm_commander

    # Move to named target
    ros2 param set /arm_commander target_name cluster_1
    ros2 service call /go_to_named std_srvs/srv/SetBool "{data: true}"

    # Move to custom position
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
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import yaml
import os


class ArmCommander(Node):
    """Arm commander using Cartesian goals with MoveIt IK."""

    def __init__(self):
        super().__init__('arm_commander')

        self.callback_group = ReentrantCallbackGroup()

        # Parameter for config file
        self.declare_parameter('config_file', '')

        # Parameters for custom target position
        self.declare_parameter('target_x', 0.75)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.45)
        self.declare_parameter('target_name', '')

        # Load named targets from config file
        self.named_targets = self.load_targets_from_config()

        # Add home position (always available)
        if 'home' not in self.named_targets:
            self.named_targets['home'] = [0.3, 0.0, 0.6]  # Safe home (forward and up)

        # Joint names for the "arm" planning group (4 joints only, no gripper)
        self.arm_joint_names = ['hip', 'shoulder', 'elbow', 'wrist']

        # Planning frame (use base_link for MoveIt)
        self.planning_frame = "world"  # or "base_link" if preferred

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
        self.get_logger().info("ARM COMMANDER READY (Cartesian IK Mode)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Loaded {len(self.named_targets)} targets from config:")
        for name, pos in self.named_targets.items():
            self.get_logger().info(f"  {name}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        self.get_logger().info("-" * 60)
        self.get_logger().info("Usage (named target):")
        self.get_logger().info("  ros2 param set /arm_commander target_name cluster_1")
        self.get_logger().info("  ros2 service call /go_to_named std_srvs/srv/SetBool \"{data: true}\"")
        self.get_logger().info("-" * 60)
        self.get_logger().info("Usage (custom position):")
        self.get_logger().info("  ros2 param set /arm_commander target_x 0.75")
        self.get_logger().info("  ros2 param set /arm_commander target_y -0.45")
        self.get_logger().info("  ros2 param set /arm_commander target_z 0.42")
        self.get_logger().info("  ros2 service call /go_to_pose std_srvs/srv/SetBool \"{data: true}\"")
        self.get_logger().info("=" * 60)

    def load_targets_from_config(self):
        """Load named targets from environment_config.yaml."""
        config_file = self.get_parameter('config_file').value

        if not config_file:
            # Try default path
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_path = get_package_share_directory('robot_arm')
                config_file = os.path.join(pkg_path, 'config', 'environment_config.yaml')
            except Exception:
                self.get_logger().warn("Could not find default config, using hardcoded targets")
                return self._fallback_targets()

        if not os.path.exists(config_file):
            self.get_logger().warn(f"Config file not found: {config_file}")
            return self._fallback_targets()

        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            targets = {}

            # Load clusters
            for name, data in config.get('clusters', {}).items():
                targets[name] = data['position']

            # Load landmarks
            for name, data in config.get('landmarks', {}).items():
                targets[name] = data['position']

            self.get_logger().info(f"Loaded targets from: {config_file}")
            return targets

        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            return self._fallback_targets()

    def _fallback_targets(self):
        """Fallback hardcoded targets if config not available."""
        return {
            'cluster_1': [0.75, -0.45, 0.42],
            'cluster_2': [0.85, 0.0, 0.52],
            'cluster_3': [0.75, 0.45, 0.46],
            'reservoir': [0.0, 0.6, 0.2],
            'home': [0.3, 0.0, 0.6],
        }

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
        """Service callback for named targets - uses Cartesian IK goals."""
        if not request.data:
            response.success = False
            response.message = "Set data=true to trigger"
            return response

        target_name = self.get_parameter('target_name').value

        # SPECIAL CASE: 'home' uses joint goal for reliability
        if target_name == 'home':
            # Use "home" pose: Retracted safe position (pulled back)
            # Hip=0, Shoulder=-1.0 (back), Elbow=1.8 (folded), Wrist=0
            joint_values = [0.0, -1.0, 1.8, 0.0]
            self.get_logger().info(f"Moving to '{target_name}' using JOINT goal: {joint_values}")
            success = self.send_joint_goal(joint_values)
            response.success = success
            response.message = f"{'Success' if success else 'Failed'}: {target_name} (Joint Goal)"
            return response

        if target_name not in self.named_targets:
            response.success = False
            response.message = f"Unknown target: {target_name}. Available: {list(self.named_targets.keys())}"
            return response

        # Targets are now [x, y, z] lists from config
        pos = self.named_targets[target_name]
        x, y, z = pos[0], pos[1], pos[2]

        self.get_logger().info(f"Moving to '{target_name}' at ({x:.2f}, {y:.2f}, {z:.2f})")

        # Use Cartesian IK goal (let MoveIt solve IK)
        success = self.send_pose_goal(x, y, z)

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
        sphere.dimensions = [0.08]  # 8cm tolerance for easier sampling
        bounding_volume.primitives.append(sphere)

        primitive_pose = Pose()
        primitive_pose.position = target_point
        primitive_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(primitive_pose)

        position_constraint.constraint_region = bounding_volume
        goal_constraints.position_constraints.append(position_constraint)

        # NOTE: Do NOT add orientation constraints for 4-DOF arm
        # The arm can only control position, not orientation
        # KDL position_only_ik mode handles this automatically

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
