#!/usr/bin/env python3
"""
Arm Commander Node - IK + Joint Goal Approach

Computes IK for target pose (position + orientation), validates the solution,
then sends joint-space goal. No OMPL sampling — predictable, fast, reliable.

Services:
    /go_to_pose (std_srvs/SetBool) - Move to target_x/y/z position
    /go_to_named (std_srvs/SetBool) - Move to named target from config

Usage:
    ros2 param set /arm_commander target_name cluster_1
    ros2 service call /go_to_named std_srvs/srv/SetBool "{data: true}"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, RobotState
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import yaml
import os
import math
import tf2_ros


class ArmCommander(Node):
    """Arm commander using IK + joint goals."""

    HOME_JOINTS = [0.0, -0.922, 2.4494, 0.0, -1.5708, 0.0]

    def __init__(self):
        super().__init__('arm_commander')

        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('target_x', 0.75)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.45)
        self.declare_parameter('target_name', '')

        # Config
        self.named_targets = self.load_targets_from_config()
        if 'home' not in self.named_targets:
            self.named_targets['home'] = [0.3, 0.0, 0.6]

        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.planning_frame = "world"

        # Current joint states (updated by subscriber)
        self.current_joints = {}
        self.joints_received = False
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # MoveGroup action client (for joint goals)
        self._action_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=self.callback_group)

        # IK service client
        self._ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.callback_group)

        # Services
        self.create_service(SetBool, 'go_to_pose', self.go_to_pose_callback,
                            callback_group=self.callback_group)
        self.create_service(SetBool, 'go_to_named', self.go_to_named_callback,
                            callback_group=self.callback_group)

        # Wait for servers
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            return
        self.get_logger().info("Waiting for /compute_ik service...")
        if not self._ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/compute_ik service not available!")
            return
        self.get_logger().info("Connected to MoveGroup + IK service!")

        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        import time
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joints_received:
                break
            time.sleep(0.1)
        if self.joints_received:
            self.get_logger().info(f"Joint states received: {list(self.current_joints.keys())}")
        else:
            self.get_logger().warn("Joint states not received yet, proceeding anyway")

        # Home on startup
        self.get_logger().info("Moving to HOME position...")
        if self.send_joint_goal(self.HOME_JOINTS):
            self.get_logger().info("HOME position reached!")
        else:
            self.get_logger().warn("Failed to reach HOME position")

        # Print targets
        self.get_logger().info("=" * 60)
        self.get_logger().info("ARM COMMANDER READY (IK + Joint Goal Mode)")
        self.get_logger().info("=" * 60)
        for name, pos in self.named_targets.items():
            self.get_logger().info(f"  {name}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        self.get_logger().info("=" * 60)

    # ─── Joint state tracking ──────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.arm_joint_names:
                self.current_joints[name] = pos
        if len(self.current_joints) >= 6 and not self.joints_received:
            self.joints_received = True

    def _get_current_joint_values(self):
        """Return current joint values as list, ordered by arm_joint_names."""
        return [self.current_joints.get(n, 0.0) for n in self.arm_joint_names]

    # ─── Config loading ────────────────────────────────────────

    def load_targets_from_config(self):
        config_file = self.get_parameter('config_file').value
        if not config_file:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_path = get_package_share_directory('robot_arm')
                config_file = os.path.join(pkg_path, 'config', 'environment_config.yaml')
            except Exception:
                return self._fallback_targets()
        if not os.path.exists(config_file):
            return self._fallback_targets()
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            targets = {}
            for name, data in config.get('clusters', {}).items():
                targets[name] = data['position']
            for name, data in config.get('landmarks', {}).items():
                targets[name] = data['position']
            self.get_logger().info(f"Loaded targets from: {config_file}")
            return targets
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            return self._fallback_targets()

    def _fallback_targets(self):
        return {
            'cluster_1': [0.875, 0.475, 0.46],
            'cluster_2': [0.975, 0.0, 0.52],
            'cluster_3': [0.875, -0.475, 0.42],
            'reservoir': [0.0, 0.6, 0.2],
            'home': [0.3, 0.0, 0.6],
        }

    # ─── Orientation computation ───────────────────────────────

    def compute_approach_quaternion(self, x, y, z):
        """Quaternion that points tcp Z-axis horizontally from base toward target."""
        dx = x - 0.0  # base X
        dy = y - 0.0  # base Y
        # Horizontal approach (dz=0)
        length = math.sqrt(dx * dx + dy * dy)
        if length < 0.01:
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        dx /= length
        dy /= length

        # Quaternion rotating (0,0,1) → (dx, dy, 0)
        qx = -dy
        qy = dx
        qz = 0.0
        qw = 1.0  # dot((0,0,1),(dx,dy,0)) = 0, so qw = 1 + 0 = 1

        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        return Quaternion(x=qx / norm, y=qy / norm, z=qz / norm, w=qw / norm)

    # ─── IK computation ───────────────────────────────────────

    def compute_ik(self, x, y, z, orientation=None):
        """Call MoveIt /compute_ik with current joint state as seed.

        Returns joint values list on success, None on failure.
        """
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.ik_link_name = "tcp"  # Solve for tcp frame, not tool0
        req.ik_request.avoid_collisions = True

        # Target pose
        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        if orientation:
            pose.pose.orientation = orientation
        else:
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        req.ik_request.pose_stamped = pose

        # Seed state = current joint values (CRITICAL for getting nearest solution)
        seed = RobotState()
        seed.joint_state.name = list(self.arm_joint_names)
        seed.joint_state.position = self._get_current_joint_values()
        req.ik_request.robot_state = seed

        self.get_logger().info(
            f"[IK] Requesting IK: target=({x:.3f}, {y:.3f}, {z:.3f}), "
            f"seed={[f'{v:.2f}' for v in seed.joint_state.position]}")
        if orientation:
            self.get_logger().info(
                f"[IK] Orientation: q=({orientation.x:.3f}, {orientation.y:.3f}, "
                f"{orientation.z:.3f}, {orientation.w:.3f})")

        # Call service
        future = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        result = future.result()
        if result is None:
            self.get_logger().error("[IK] Service call failed (timeout)")
            return None

        if result.error_code.val != 1:
            self.get_logger().error(f"[IK] IK failed: error_code={result.error_code.val}")
            return None

        # Extract joint values
        joint_values = []
        for name in self.arm_joint_names:
            idx = list(result.solution.joint_state.name).index(name)
            joint_values.append(result.solution.joint_state.position[idx])

        self.get_logger().info(
            f"[IK] Raw solution: {[f'{v:.3f}' for v in joint_values]}")

        # Normalize joints to be closest to current (avoid full rotations)
        joint_values = self._normalize_joints(joint_values)
        self.get_logger().info(
            f"[IK] Normalized:   {[f'{v:.3f}' for v in joint_values]}")

        return joint_values

    def _normalize_joints(self, joint_values):
        """Wrap each joint to be within π of current value (shortest path)."""
        current = self._get_current_joint_values()
        normalized = []
        for target, curr in zip(joint_values, current):
            while target - curr > math.pi:
                target -= 2 * math.pi
            while target - curr < -math.pi:
                target += 2 * math.pi
            normalized.append(target)
        return normalized

    def validate_joints(self, joint_values):
        """Sanity check: are joint values reasonable?"""
        j1 = joint_values[0]
        # joint1 should be in front hemisphere (-135° to 135°)
        if abs(j1) > math.pi * 0.75:
            self.get_logger().warn(
                f"[VALIDATE] joint1={j1:.2f} rad ({math.degrees(j1):.0f}°) — arm may be reaching backward!")
            return False
        return True

    # ─── TF debug ──────────────────────────────────────────────

    def _log_tcp_position(self, label=""):
        try:
            t = self._tf_buffer.lookup_transform('world', 'tcp', rclpy.time.Time())
            p = t.transform.translation
            r = t.transform.rotation
            self.get_logger().info(
                f"  [TCP] {label} pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
                f"rot=({r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f})")
        except Exception as e:
            self.get_logger().warn(f"  [TCP] {label} TF failed: {e}")

    # ─── Service callbacks ─────────────────────────────────────

    def go_to_pose_callback(self, request, response):
        if not request.data:
            response.success = False
            response.message = "Set data=true to trigger"
            return response

        x = self.get_parameter('target_x').value
        y = self.get_parameter('target_y').value
        z = self.get_parameter('target_z').value

        success = self.move_to_pose(x, y, z)
        response.success = success
        response.message = f"{'OK' if success else 'FAIL'}: ({x}, {y}, {z})"
        return response

    def go_to_named_callback(self, request, response):
        if not request.data:
            response.success = False
            response.message = "Set data=true to trigger"
            return response

        target_name = self.get_parameter('target_name').value

        # Home = direct joint goal
        if target_name == 'home':
            self.get_logger().info("Moving to HOME (joint goal)")
            success = self.send_joint_goal(self.HOME_JOINTS)
            response.success = success
            response.message = f"{'OK' if success else 'FAIL'}: home"
            return response

        if target_name not in self.named_targets:
            response.success = False
            response.message = f"Unknown: {target_name}. Available: {list(self.named_targets.keys())}"
            return response

        pos = self.named_targets[target_name]
        x, y, z = pos[0], pos[1], pos[2]
        use_orientation = target_name.startswith('cluster')

        self.get_logger().info(f"Moving to '{target_name}' at ({x:.3f}, {y:.3f}, {z:.3f}), "
                               f"approach_orientation={use_orientation}")

        success = self.move_to_pose(x, y, z, approach_orientation=use_orientation)
        response.success = success
        response.message = f"{'OK' if success else 'FAIL'}: {target_name}"
        return response

    # ─── Core: IK → validate → joint goal ─────────────────────

    def move_to_pose(self, x, y, z, approach_orientation=False):
        """Compute IK for target pose, validate, send joint goal."""
        self._log_tcp_position("BEFORE move")

        # Compute orientation
        orientation = None
        if approach_orientation:
            orientation = self.compute_approach_quaternion(x, y, z)

        # Compute IK (seed = current joints)
        joint_values = self.compute_ik(x, y, z, orientation=orientation)
        if joint_values is None:
            self.get_logger().error("IK failed — cannot reach target")
            return False

        # Validate
        if not self.validate_joints(joint_values):
            self.get_logger().error("Joint validation failed — solution looks wrong, aborting")
            return False

        # Send joint goal
        self.get_logger().info(f"Sending joint goal from IK solution...")
        success = self.send_joint_goal(joint_values)

        if success:
            self._log_tcp_position(f"AFTER move (target was {x:.3f},{y:.3f},{z:.3f})")
            # Error check
            try:
                t = self._tf_buffer.lookup_transform('world', 'tcp', rclpy.time.Time())
                p = t.transform.translation
                err = math.sqrt((p.x-x)**2 + (p.y-y)**2 + (p.z-z)**2)
                self.get_logger().info(f"  Position error: {err:.4f}m")
                if err > 0.05:
                    self.get_logger().warn(f"  Position error > 5cm ({err:.3f}m)")
            except Exception:
                pass

        return success

    # ─── Joint goal execution ──────────────────────────────────

    def send_joint_goal(self, joint_values):
        """Send joint-space goal via MoveGroup action."""
        from moveit_msgs.msg import JointConstraint

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 1.0
        goal_msg.request.max_acceleration_scaling_factor = 1.0

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
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        self.get_logger().info(f"Joint goal: {[f'{v:.3f}' for v in joint_values]}")

        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Joint goal rejected!")
            return False

        self.get_logger().info("Goal accepted, executing...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error("Execution timeout!")
            return False

        if result.result.error_code.val == 1:
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
