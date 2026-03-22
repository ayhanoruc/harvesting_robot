"""
Arm Commander Node - Joint Space Version

Moves the robot to named target positions using joint-space goals.
Much more reliable for simple 2-DOF arms than Cartesian goals.

Services:
    /go_to_target (std_srvs/SetBool) - Move to current target parameter

Usage:
    ros2 run example_arm_moveit_config arm_commander
    ros2 param set /arm_commander target HOME
    ros2 service call /go_to_target std_srvs/srv/SetBool "{data: true}"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

import yaml
import os
from ament_index_python.packages import get_package_share_directory


class ArmCommander(Node):
    """Arm commander using joint-space goals."""

    def __init__(self):
        super().__init__('arm_commander')

        self.callback_group = ReentrantCallbackGroup()

        # Joint names (must match URDF)
        self.joint_names = ['slider_joint', 'arm_joint']

        # Load targets
        self.targets = self.load_targets()
        self.get_logger().info(f"Loaded {len(self.targets)} targets")

        # Parameter for current target
        self.declare_parameter('target', 'HOME')

        # MoveGroup action client
        self._action_client = ActionClient(
            self,
            MoveGroup,
            'move_action',
            callback_group=self.callback_group
        )

        # Service
        self.create_service(
            SetBool,
            'go_to_target',
            self.go_to_target_callback,
            callback_group=self.callback_group
        )

        # Wait for action server
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Connected!")

        # Print info
        self.get_logger().info("=" * 50)
        self.get_logger().info("ARM COMMANDER READY (Joint Space Mode)")
        self.get_logger().info("=" * 50)
        self.get_logger().info("Available targets:")
        for name, target in self.targets.items():
            joints = target.get('joints', [0, 0])
            self.get_logger().info(f"  {name}: joints={joints}")
        self.get_logger().info("-" * 50)
        self.get_logger().info("Usage:")
        self.get_logger().info("  ros2 param set /arm_commander target <NAME>")
        self.get_logger().info("  ros2 service call /go_to_target std_srvs/srv/SetBool \"{data: true}\"")
        self.get_logger().info("=" * 50)

    def load_targets(self):
        """Load target positions from YAML."""
        try:
            pkg_path = get_package_share_directory('example_arm_moveit_config')
            yaml_path = os.path.join(pkg_path, 'config', 'target_positions.yaml')

            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)

            return config.get('targets', {})
        except Exception as e:
            self.get_logger().error(f"Failed to load targets: {e}")
            return {
                'HOME': {'joints': [0.0, 0.0]},
            }

    def go_to_target_callback(self, request, response):
        """Service callback."""
        if not request.data:
            response.success = False
            response.message = "Set data=true to trigger"
            return response

        target_name = self.get_parameter('target').value

        if target_name not in self.targets:
            response.success = False
            response.message = f"Unknown target: {target_name}"
            return response

        target = self.targets[target_name]
        joints = target.get('joints', [0.0, 0.0])

        self.get_logger().info(f"Moving to: {target_name}")
        self.get_logger().info(f"  Joint values: {joints}")

        success = self.send_joint_goal(joints)

        response.success = success
        response.message = f"{'Success' if success else 'Failed'}: {target_name}"
        return response

    def send_joint_goal(self, joint_values):
        """Send joint-space goal to MoveGroup."""
        goal_msg = MoveGroup.Goal()

        # Motion plan request
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # Joint constraints (goal)
        goal_constraints = Constraints()

        for i, (name, value) in enumerate(zip(self.joint_names, joint_values)):
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
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False

        self.get_logger().info("Goal accepted, waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        result = result_future.result()
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
