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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import yaml
import os
import math
import tf2_ros


class ArmCommander(Node):
    """Arm commander using IK + joint goals."""

    HOME_JOINTS = [0.0000, -0.922, 2.4494, 0.0, -1.3000, 0.0]

    # M1013 URDF position limits (radians). Critical: joint3 is restricted!
    # KDL IK plugin sometimes returns solutions wrapped beyond these — we
    # post-filter and reject any IK candidate that doesn't fit.
    JOINT_LIMITS = [
        (-6.2832, 6.2832),   # joint1
        (-6.2832, 6.2832),   # joint2
        (-2.7925, 2.7925),   # joint3 — RESTRICTED (~±160°)
        (-6.2832, 6.2832),   # joint4
        (-6.2832, 6.2832),   # joint5
        (-6.2832, 6.2832),   # joint6
    ]
    JOINT_LIMIT_TOL = 0.02   # rad slack for floating-point safety

    def __init__(self):
        super().__init__('arm_commander')

        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('target_x', 0.75)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.45)
        self.declare_parameter('target_name', '')
        self.declare_parameter('pre_grasp_offset', 0.15)  # meters back from boll along approach
        self.declare_parameter('use_approach_orientation', False)  # set by orchestrator before go_to_pose
        self.declare_parameter('cluster_rotate_deg', 90.0)  # step-2: rotate wrist joint before gripper (joint5)
        self.declare_parameter('use_direct_trajectory', False)  # bypass MoveGroup, direct joint traj

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

        # MoveGroup action client (for HOME, rotation — internal use)
        self._action_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=self.callback_group)

        # Direct joint trajectory publisher (for pipeline moves — bypasses OMPL)
        self._traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # IK service client
        self._ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.callback_group)

        # Services
        self.create_service(SetBool, 'go_to_pose', self.go_to_pose_callback,
                            callback_group=self.callback_group)
        self.create_service(SetBool, 'go_to_named', self.go_to_named_callback,
                            callback_group=self.callback_group)
        self.create_service(SetBool, 'go_home_view', self.go_home_view_callback,
                            callback_group=self.callback_group)
        self.create_service(SetBool, 'rotate_home_view_to_clusters', self.rotate_home_view_to_clusters_callback,
                            callback_group=self.callback_group)
        self.create_service(SetBool, 'go_to_reservoir', self.go_to_reservoir_callback,
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
            # Immediately run step-2 on startup as requested.
            delta_deg = float(self.get_parameter('cluster_rotate_deg').value)
            self._rotate_joint5_by_delta_deg(delta_deg)
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
        """Quaternion that points tcp Z-axis horizontally from arm base toward target.

        Looks up base_0 → world via TF so it works on a mobile platform (Husky).
        Falls back to origin (legacy fixed-base behavior) if TF unavailable.
        """
        base_x, base_y = 0.0, 0.0
        try:
            t = self._tf_buffer.lookup_transform(
                self.planning_frame, 'base_0', rclpy.time.Time())
            base_x = t.transform.translation.x
            base_y = t.transform.translation.y
        except Exception as e:
            self.get_logger().warn(
                f'[approach_quat] TF base_0 lookup failed ({e}); using origin')

        dx = x - base_x
        dy = y - base_y
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

    def compute_ik_multi_seed(self, x, y, z, orientation=None):
        """Try IK with HOME seed and current seed, pick best solution.

        Returns joint values list on success, None on failure.
        """
        if orientation:
            self.get_logger().info(
                f"[IK] Target: ({x:.3f}, {y:.3f}, {z:.3f}), "
                f"q=({orientation.x:.3f}, {orientation.y:.3f}, "
                f"{orientation.z:.3f}, {orientation.w:.3f})")

        current_joints = self._get_current_joint_values()
        candidates = []

        # Seed 1: HOME (reliable, neutral starting point)
        home_result = self._ik_call(x, y, z, orientation, self.HOME_JOINTS, "HOME")
        if home_result is not None:
            candidates.append(home_result)

        # Seed 2: current joints (shorter path if already nearby)
        curr_result = self._ik_call(x, y, z, orientation, current_joints, "CURRENT")
        if curr_result is not None:
            candidates.append(curr_result)

        if not candidates:
            self.get_logger().error("[IK] Both seeds failed — target unreachable")
            return None

        # Pick candidate with smallest total joint change from current position
        best = None
        best_cost = float('inf')
        for jv in candidates:
            cost = sum(abs(a - b) for a, b in zip(jv, current_joints))
            if cost < best_cost:
                best_cost = cost
                best = jv

        self.get_logger().info(
            f"[IK] Best solution (cost={best_cost:.2f} rad): "
            f"{[f'{v:.3f}' for v in best]}")

        return best

    def _ik_call(self, x, y, z, orientation, seed_joints, seed_label):
        """Single IK call with given seed. Returns normalized joint values or None."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.ik_link_name = "tcp"
        req.ik_request.avoid_collisions = True

        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        pose.pose.orientation = orientation if orientation else Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        req.ik_request.pose_stamped = pose

        seed = RobotState()
        seed.joint_state.name = list(self.arm_joint_names)
        seed.joint_state.position = list(seed_joints)
        req.ik_request.robot_state = seed

        self.get_logger().info(
            f"[IK] Seed={seed_label}: {[f'{v:.2f}' for v in seed_joints]}")

        future = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        result = future.result()
        if result is None or result.error_code.val != 1:
            self.get_logger().warn(f"[IK] Seed={seed_label} failed")
            return None

        joint_values = []
        for name in self.arm_joint_names:
            idx = list(result.solution.joint_state.name).index(name)
            joint_values.append(result.solution.joint_state.position[idx])

        joint_values, all_ok = self._normalize_joints(joint_values)
        if not all_ok:
            self.get_logger().warn(
                f"[IK] Seed={seed_label} solution outside URDF limits, rejecting")
            return None
        self.get_logger().info(
            f"[IK] Seed={seed_label} solution: {[f'{v:.3f}' for v in joint_values]}")

        return joint_values

    def _normalize_joints(self, joint_values):
        """Wrap each joint into URDF limits, preferring the value closest to current.

        Strategy: for each joint generate candidates {target ± k·2π for k=-2..2},
        keep only those within URDF limits, then pick the one closest to the
        current joint angle. This both respects physical limits AND minimizes
        path length, avoiding the prior bug where shortest-path-from-current
        could land outside joint3's ±160° range.

        Returns (normalized_joints, all_within_limits).
        """
        current = self._get_current_joint_values()
        normalized = []
        all_ok = True
        for i, (target, curr, (lo, hi)) in enumerate(
                zip(joint_values, current, self.JOINT_LIMITS)):
            tol = self.JOINT_LIMIT_TOL
            candidates = [target + k * 2 * math.pi for k in (-2, -1, 0, 1, 2)]
            in_lim = [c for c in candidates if (lo - tol) <= c <= (hi + tol)]
            if not in_lim:
                self.get_logger().warn(
                    f'[NORMALIZE] joint{i+1} no wrap fits limits '
                    f'[{lo:.2f},{hi:.2f}] (raw={target:.2f})')
                all_ok = False
                normalized.append(max(lo, min(hi, target)))  # clamp as fallback
            else:
                # Pick within-limit candidate closest to current pose
                best = min(in_lim, key=lambda c: abs(c - curr))
                normalized.append(best)
        return normalized, all_ok

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
        approach = self.get_parameter('use_approach_orientation').value

        success = self.move_to_pose(x, y, z, approach_orientation=approach)
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

        # Pre-grasp offset for cluster targets
        if use_orientation:
            offset = self.get_parameter('pre_grasp_offset').value
            dx, dy = x - 0.0, y - 0.0  # from base
            length = math.sqrt(dx * dx + dy * dy)
            if length > 0.01:
                x -= offset * (dx / length)
                y -= offset * (dy / length)
            self.get_logger().info(
                f"Moving to '{target_name}' PRE-GRASP at ({x:.3f}, {y:.3f}, {z:.3f}), "
                f"offset={offset}m from boll ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        else:
            self.get_logger().info(f"Moving to '{target_name}' at ({x:.3f}, {y:.3f}, {z:.3f})")

        success = self.move_to_pose(x, y, z, approach_orientation=use_orientation)
        response.success = success
        response.message = f"{'OK' if success else 'FAIL'}: {target_name}"
        return response

    def go_home_view_callback(self, request, response):
        """Go to HOME, then immediately rotate joint5 toward clusters."""
        if not request.data:
            response.success = False
            response.message = "Set data=true to trigger"
            return response

        self.get_logger().info("[STEP-1] Moving to HOME view")
        home_ok = self.send_joint_goal(self.HOME_JOINTS)
        if not home_ok:
            response.success = False
            response.message = "FAIL: home_view"
            return response

        delta_deg = float(self.get_parameter('cluster_rotate_deg').value)
        rot_ok = self._rotate_joint5_by_delta_deg(delta_deg)
        response.success = rot_ok
        response.message = f"{'OK' if rot_ok else 'FAIL'}: home_then_rotate_{delta_deg:.1f}deg"
        return response

    def rotate_home_view_to_clusters_callback(self, request, response):
        """Step-2 only: rotate wrist joint before gripper (joint5) by configured angle."""
        if not request.data:
            response.success = False
            response.message = "Set data=true to trigger"
            return response

        delta_deg = float(self.get_parameter('cluster_rotate_deg').value)
        success = self._rotate_joint5_by_delta_deg(delta_deg)
        response.success = success
        response.message = f"{'OK' if success else 'FAIL'}: rotate_joint5_{delta_deg:.1f}deg"
        return response

    def _rotate_joint5_by_delta_deg(self, delta_deg: float) -> bool:
        """Rotate only joint5 (wrist before gripper) by given delta degrees."""
        current = self._get_current_joint_values()
        target = list(current)
        j5_idx = self.arm_joint_names.index('joint5')
        target[j5_idx] = current[j5_idx] + math.radians(delta_deg)  # joint5 only

        self.get_logger().info(
            f"[STEP-2] Rotating joint5 only: {math.degrees(current[j5_idx]):.1f} deg -> "
            f"{math.degrees(target[j5_idx]):.1f} deg (delta={delta_deg:.1f} deg)")
        return self.send_joint_goal(target)

    def go_to_reservoir_callback(self, request, response):
        """Heuristic 3-stage motion to reservoir hover position.

        Reservoir sits at -X local from arm base (= behind robot in world frame
        after Husky's -π/2 spawn yaw). Forcing a single-shot IK from any pose to
        this 'behind' target produces over-the-shoulder solutions that push joint3
        toward its ±2.79 rad URDF limit. KDL often returns wrapped solutions just
        outside that limit and the trajectory either gets clamped or silently
        fails.

        Decompose the motion instead:
          Stage 1 — fold to HOME (safe transit pose, arm tucked).
          Stage 2 — rotate joint1 by π (keeping joints 2–6 at HOME values).
                    Now the arm physically faces toward reservoir.
          Stage 3 — IK to reservoir hover. From this rotated pose IK is a normal
                    'front-of-arm' reach; joint3 lands mid-range, no limit fight.
        """
        if not request.data:
            response.success = False
            response.message = "Set data=true to trigger"
            return response

        # Stage 1: HOME for safe transit
        self.get_logger().info("[RESERVOIR] Stage 1/3: fold to HOME")
        if not self.send_joint_goal(self.HOME_JOINTS):
            response.success = False
            response.message = "FAIL: HOME fold"
            return response

        # Stage 2: rotate joint1 to ±π (face local -X = reservoir direction).
        # Pick whichever sign is closer to current joint1 to take the short path.
        current = self._get_current_joint_values()
        target_j1 = math.pi if current[0] >= 0.0 else -math.pi
        rotated = list(current)
        rotated[0] = target_j1
        self.get_logger().info(
            f"[RESERVOIR] Stage 2/3: rotate joint1 {current[0]:.2f} → {target_j1:.2f} rad "
            f"(other joints unchanged)")
        if not self.send_joint_goal(rotated):
            response.success = False
            response.message = "FAIL: joint1 rotation"
            return response

        # Stage 3: IK to reservoir hover position. Use TF (works on mobile base).
        hover_m = 0.30
        try:
            t = self._tf_buffer.lookup_transform(
                self.planning_frame, 'reservoir_link', rclpy.time.Time())
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            rz = t.transform.translation.z + hover_m
        except Exception as e:
            self.get_logger().error(f"[RESERVOIR] reservoir_link TF lookup failed: {e}")
            response.success = False
            response.message = "FAIL: reservoir TF"
            return response

        self.get_logger().info(
            f"[RESERVOIR] Stage 3/3: IK to hover ({rx:.3f}, {ry:.3f}, {rz:.3f})")
        success = self.move_to_pose(rx, ry, rz, approach_orientation=True)
        response.success = success
        response.message = (
            f"OK: reservoir hover ({rx:.2f},{ry:.2f},{rz:.2f})"
            if success else "FAIL: IK to reservoir from rotated pose")
        return response

    # ─── Core: IK → joint goal ─────────────────────────────────

    def move_to_pose(self, x, y, z, approach_orientation=False):
        """Compute IK for target pose, validate, send joint goal.
        If position error > 5cm after execution, retries via HOME fallback.
        """
        self._log_tcp_position("BEFORE move")

        # Compute orientation
        orientation = None
        if approach_orientation:
            orientation = self.compute_approach_quaternion(x, y, z)

        # Compute IK (multi-seed: HOME + current). _normalize_joints already
        # rejects any candidate that doesn't fit URDF limits, so anything we
        # get back here is safe to send.
        joint_values = self.compute_ik_multi_seed(x, y, z, orientation=orientation)
        if joint_values is None:
            self.get_logger().error("IK failed — cannot reach target")
            return False

        # Send joint goal — direct trajectory or MoveGroup depending on param
        use_direct = self.get_parameter('use_direct_trajectory').value
        if use_direct:
            self.get_logger().info("Sending joint goal (DIRECT trajectory)...")
            success = self.send_joint_goal_direct(joint_values)
        else:
            self.get_logger().info("Sending joint goal (MoveGroup)...")
            success = self.send_joint_goal(joint_values)

        if not success:
            return False

        # Check position error
        err = self._check_tcp_error(x, y, z)

        # Fallback: if error > 5cm, go HOME and retry
        if err is not None and err > 0.05:
            self.get_logger().warn(f"[RETRY] Error {err:.3f}m > 5cm — going HOME then retrying")
            home_ok = self.send_joint_goal(self.HOME_JOINTS)
            if not home_ok:
                self.get_logger().error("[RETRY] Failed to reach HOME")
                return False

            # Re-compute IK from HOME position (HOME seed will dominate)
            joint_values = self.compute_ik_multi_seed(x, y, z, orientation=orientation)
            if joint_values is None:
                self.get_logger().error("[RETRY] IK failed after HOME reset")
                return False

            self.get_logger().info("[RETRY] Sending joint goal after HOME reset...")
            success = self.send_joint_goal(joint_values)
            if success:
                err2 = self._check_tcp_error(x, y, z)
                if err2 is not None and err2 > 0.05:
                    self.get_logger().warn(f"[RETRY] Still off: {err2:.3f}m")
                    return False
            else:
                return False

        return True

    def _check_tcp_error(self, x, y, z):
        """Check TCP position error after move. Returns error in meters or None."""
        try:
            t = self._tf_buffer.lookup_transform('world', 'tcp', rclpy.time.Time())
            p = t.transform.translation
            r = t.transform.rotation
            err = math.sqrt((p.x - x)**2 + (p.y - y)**2 + (p.z - z)**2)
            self.get_logger().info(
                f"  [TCP] AFTER pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
                f"rot=({r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f})")
            self.get_logger().info(f"  Position error: {err:.4f}m")
            return err
        except Exception as e:
            self.get_logger().warn(f"  [TCP] TF lookup failed: {e}")
            return None

    # ─── Joint goal execution ──────────────────────────────────

    def send_joint_goal(self, joint_values):
        """Send joint-space goal via MoveGroup action."""
        from moveit_msgs.msg import JointConstraint

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 3
        goal_msg.request.allowed_planning_time = 3.0
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

    def send_joint_goal_direct(self, joint_values, duration_sec=1.5):
        """Send joint-space goal via direct JointTrajectory publish (no OMPL).

        Same pattern as explorer._move_to_joints_sync: publish and sleep.
        No convergence polling — just wait enough wall-clock time for sim.
        """
        self.get_logger().info(f"Joint goal (direct): {[f'{v:.3f}' for v in joint_values]}")

        traj_msg = JointTrajectory()
        traj_msg.joint_names = list(self.arm_joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(joint_values[:6])
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9))
        traj_msg.points = [point]

        self._traj_pub.publish(traj_msg)

        # Wait like explorer does: duration + buffer
        import time
        time.sleep(duration_sec + 0.5)

        self.get_logger().info("Direct trajectory sent and waited.")
        return True


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

