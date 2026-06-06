#!/usr/bin/env python3
"""
Gripper Controller Node — Open/Close services for the simple box gripper.

Single moving joint: hande_left_finger_joint (prismatic, range 0..0.030).
Right finger is a fixed decorative box.

Pipeline:
  /gripper/open  → publish target on /gripper_controller/commands → poll
                   /joint_states until within tolerance OR timeout.
  /gripper/close → same.

Success is reported ONLY if the joint actually reached the target — no
blind success.

Uses MultiThreadedExecutor so the joint_state subscription callback can
fire concurrently while the service callback polls for convergence.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class GripperController(Node):

    def __init__(self):
        super().__init__('gripper_controller_node')

        self.cb_group = ReentrantCallbackGroup()

        # Simple box gripper, prismatic left finger (URDF: hande_left_finger_joint):
        #   joint 0.000 → left pad at -X end of palm = OPEN  (~30mm gap)
        #   joint 0.022 → CLOSED with a small ~8mm gap (visible, not fully touching).
        #   joint 0.030 → fully closed (pads touching) — kept reachable as the joint
        #                 upper limit, but not used by /gripper/close.
        self.declare_parameter('open_position', 0.0)
        self.declare_parameter('close_position', 0.022)
        self.declare_parameter('position_tolerance', 0.002)
        self.declare_parameter('settle_timeout', 3.0)  # seconds (wall-clock;
        # bumped from 1.5 → 3.0 so a low real-time-factor dip during a pick
        # doesn't time out the finger before it travels the full 22 mm and
        # /joint_states reports it — the old 1.5 s caused spurious close
        # failures that "fixed themselves" on the retry. Normal close settles
        # in ~0.6 s, so 3 s is comfortable headroom without blocking long on a
        # genuine failure.)

        self.open_pos = float(self.get_parameter('open_position').value)
        self.close_pos = float(self.get_parameter('close_position').value)
        self.tolerance = float(self.get_parameter('position_tolerance').value)
        self.timeout = float(self.get_parameter('settle_timeout').value)

        self.joint_name = 'hande_left_finger_joint'
        self.current_position = None  # last seen joint position

        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10,
            callback_group=self.cb_group)

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)

        self.create_service(
            Trigger, '/gripper/open', self._open_cb,
            callback_group=self.cb_group)
        self.create_service(
            Trigger, '/gripper/close', self._close_cb,
            callback_group=self.cb_group)

        self.get_logger().info('Gripper Controller ready.')
        self.get_logger().info(f'  /gripper/open  → {self.open_pos}m')
        self.get_logger().info(f'  /gripper/close → {self.close_pos}m')

    def _joint_state_cb(self, msg: JointState):
        if self.joint_name in msg.name:
            idx = msg.name.index(self.joint_name)
            self.current_position = msg.position[idx]

    def _publish(self, value: float):
        msg = Float64MultiArray()
        msg.data = [value]
        self.cmd_pub.publish(msg)

    def _wait_until_at(self, target: float) -> bool:
        """Poll /joint_states until the joint is within tolerance of target,
        or timeout. Returns (reached, final |error|).

        The timeout is measured in SIM time (the node runs use_sim_time), not
        wall-clock. The finger is driven by Gazebo and /joint_states is stamped
        in sim time, so both are paced by the real-time-factor. Using the
        wall-clock here (the old `time.time()` version) meant that whenever RTF
        dipped — e.g. right after a heavy reservoir motion + OMPL plan + the
        `ign set_pose` teleport subprocesses all hammered the CPU — 1.5 s of
        wall-clock bought almost no sim time, the finger hadn't travelled the
        full 22 mm yet, and close() timed out and reported a spurious failure
        (the command persists, so the *next* attempt then 'succeeds' once the
        finger has crept into tolerance). Pacing the deadline by the ROS clock
        makes the budget RTF-independent. A wall-clock backstop (4× the budget)
        guards against a stalled /clock so we never block forever."""
        timeout = rclpy.duration.Duration(seconds=self.timeout)
        sim_deadline = self.get_clock().now() + timeout
        wall_deadline = time.time() + max(4.0 * self.timeout, 10.0)
        last_err = float('inf')
        while self.get_clock().now() < sim_deadline and time.time() < wall_deadline:
            if self.current_position is not None:
                last_err = abs(self.current_position - target)
                if last_err < self.tolerance:
                    return True, last_err
            time.sleep(0.02)
        return False, last_err

    def _move_to(self, target: float, label: str, response):
        cur = f'{self.current_position:.4f}' if self.current_position is not None else 'unknown'
        self.get_logger().info(f'{label} → {target:.4f}m (current: {cur})')
        self._publish(target)
        ok, err = self._wait_until_at(target)
        if ok:
            response.success = True
            response.message = f'Gripper {label.lower()} (err={err*1000:.2f}mm)'
            self.get_logger().info(
                f'{label} reached: pos={self.current_position:.4f}m err={err*1000:.2f}mm')
        else:
            response.success = False
            response.message = (
                f'Gripper FAILED to {label.lower()} '
                f'(pos={self.current_position}, target={target}, '
                f'err={err*1000:.2f}mm, timeout={self.timeout}s)')
            self.get_logger().warn(response.message)
        return response

    def _open_cb(self, request, response):
        return self._move_to(self.open_pos, 'opened', response)

    def _close_cb(self, request, response):
        return self._move_to(self.close_pos, 'closed', response)


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
