"""
Camera Focus Node - Simple Pixel-Error Heuristic

Centers the camera on a target pixel by tweaking joint angles.
No complex geometry - just proportional control on pixel error.

Service:
    /camera_focus/center_on_pixel (harvester_interfaces/srv/FocusFromPixel)
        - Input: u, v (pixel to center on)
        - Action: Adjusts arm joints to center that pixel
        - Output: success, message

How it works:
    1. Pixel error = (target_pixel - image_center)
    2. Convert pixel error to joint adjustments:
       - Horizontal error → hip rotation
       - Vertical error → shoulder/elbow tilt
    3. Send adjusted joint positions to MoveIt
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from harvester_interfaces.srv import FocusFromPixel

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

import math


class CameraFocusNode(Node):
    """
    Centers camera on target pixel using joint adjustments.
    """

    def __init__(self):
        super().__init__('camera_focus')
        self.get_logger().info('Camera Focus node initializing...')

        # Parameters - pixel error to joint angle gains
        self.declare_parameter('gain_hip', 0.002)        # rad per pixel
        self.declare_parameter('gain_shoulder', 0.0015)  # rad per pixel
        self.declare_parameter('gain_elbow', 0.001)      # rad per pixel
        self.declare_parameter('image_center_u', 320)    # image width / 2
        self.declare_parameter('image_center_v', 240)    # image height / 2
        self.declare_parameter('max_adjustment', 0.3)    # max radians per call

        self.gain_hip = self.get_parameter('gain_hip').value
        self.gain_shoulder = self.get_parameter('gain_shoulder').value
        self.gain_elbow = self.get_parameter('gain_elbow').value
        self.center_u = self.get_parameter('image_center_u').value
        self.center_v = self.get_parameter('image_center_v').value
        self.max_adj = self.get_parameter('max_adjustment').value

        # Joint names (must match URDF)
        self.joint_names = ['hip', 'shoulder', 'elbow', 'wrist']

        # Current joint positions
        self.current_joints = {}
        self.joints_received = False

        # Callback group for async operations
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # MoveGroup action client
        self._action_client = ActionClient(
            self,
            MoveGroup,
            'move_action',
            callback_group=self.callback_group
        )

        # Service
        self.center_srv = self.create_service(
            FocusFromPixel,
            '/camera_focus/center_on_pixel',
            self.center_on_pixel_callback,
            callback_group=self.callback_group
        )

        # Wait for MoveGroup
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
        else:
            self.get_logger().info("Connected to MoveGroup!")

        self.get_logger().info('Camera Focus ready.')
        self.get_logger().info(f'  Service: /camera_focus/center_on_pixel')
        self.get_logger().info(f'  Gains - hip: {self.gain_hip}, shoulder: {self.gain_shoulder}')

    def joint_state_callback(self, msg: JointState):
        """Store current joint positions."""
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.current_joints[name] = pos

        if len(self.current_joints) >= 4 and not self.joints_received:
            self.joints_received = True
            self.get_logger().info(f'Joint states received: {list(self.current_joints.keys())}')

    def center_on_pixel_callback(self, request, response):
        """
        Center camera on the given pixel by adjusting joints.
        """
        u, v = request.u, request.v

        self.get_logger().info(f'Center request: pixel ({u}, {v})')

        # Check if we have joint states
        if not self.joints_received:
            response.success = False
            response.message = 'Joint states not yet received'
            return response

        # Compute pixel error
        error_u = u - self.center_u  # positive = target is RIGHT
        error_v = v - self.center_v  # positive = target is DOWN

        self.get_logger().info(f'Pixel error: ({error_u}, {error_v})')

        # Compute joint adjustments
        # Hip: rotates view left/right - negative because right pixel needs left rotation
        hip_delta = -self.gain_hip * error_u

        # Shoulder: positive error_v (down) needs to tilt down (POSITIVE shoulder)
        shoulder_delta = self.gain_shoulder * error_v

        # Elbow: assists shoulder for tilt, opposite direction
        elbow_delta = -self.gain_elbow * error_v

        # Clamp adjustments
        hip_delta = max(-self.max_adj, min(self.max_adj, hip_delta))
        shoulder_delta = max(-self.max_adj, min(self.max_adj, shoulder_delta))
        elbow_delta = max(-self.max_adj, min(self.max_adj, elbow_delta))

        # Compute new joint targets
        new_joints = {
            'hip': self.current_joints.get('hip', 0.0) + hip_delta,
            'shoulder': self.current_joints.get('shoulder', 0.0) + shoulder_delta,
            'elbow': self.current_joints.get('elbow', 0.0) + elbow_delta,
            'wrist': self.current_joints.get('wrist', 0.0),  # Keep wrist same
        }

        self.get_logger().info(
            f'Joint adjustments - hip: {hip_delta:.3f}, '
            f'shoulder: {shoulder_delta:.3f}, elbow: {elbow_delta:.3f}'
        )
        self.get_logger().info(f'New joint targets: {new_joints}')

        # Execute the move
        success = self._send_joint_goal(new_joints)

        if success:
            response.success = True
            response.message = f'Centered on pixel ({u}, {v}). Adjustments: hip={hip_delta:.3f}, shoulder={shoulder_delta:.3f}'
        else:
            response.success = False
            response.message = 'Motion execution failed'

        return response

    def _send_joint_goal(self, joint_targets: dict) -> bool:
        """Send joint-space goal to MoveGroup."""
        goal_msg = MoveGroup.Goal()

        # Motion plan request
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3

        # Joint constraints
        goal_constraints = Constraints()

        for name in self.joint_names:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = joint_targets.get(name, 0.0)
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
        self.get_logger().info('Sending joint goal to MoveGroup...')

        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False

        self.get_logger().info("Goal accepted, executing...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error("Result is None!")
            return False

        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Centering complete!")
            return True
        else:
            self.get_logger().error(f"Failed: error_code={result.result.error_code.val}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = CameraFocusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
