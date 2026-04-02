#!/usr/bin/env python3
"""
Gripper Controller Node — Open/Close services for Robotiq Hand-E

Sim:  Publishes JointTrajectory to gripper_controller topic
Real: Will use Robotiq Modbus RTU driver (TODO)

Services:
    /gripper/open  (std_srvs/Trigger) — Open gripper (position=0.025m)
    /gripper/close (std_srvs/Trigger) — Close gripper (position=0.0m)

Parameters:
    open_position  — Finger position for open  (default: 0.025m, Hand-E max)
    close_position — Finger position for close (default: 0.0m)
    move_duration  — Sim-time seconds for open/close (default: 0.5s)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import time
import math


class GripperController(Node):

    def __init__(self):
        super().__init__('gripper_controller_node')

        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('open_position', 0.0)
        self.declare_parameter('close_position', 0.025)
        self.declare_parameter('move_duration', 0.2)
        self.declare_parameter('position_tolerance', 0.002)  # 2mm

        self.open_pos = self.get_parameter('open_position').value
        self.close_pos = self.get_parameter('close_position').value
        self.move_dur = self.get_parameter('move_duration').value
        self.tolerance = self.get_parameter('position_tolerance').value

        # Only left finger — right follows via URDF mimic tag
        self.joint_names = ['hande_left_finger_joint']

        # Current gripper position tracking
        self.current_position = None  # left finger
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

        # Trajectory publisher (direct topic, no action server needed)
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        # Services
        self.create_service(
            Trigger, '/gripper/open', self._open_cb,
            callback_group=self.callback_group)
        self.create_service(
            Trigger, '/gripper/close', self._close_cb,
            callback_group=self.callback_group)

        # Wait for joint states
        self.get_logger().info('Waiting for gripper joint state...')
        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_position is not None:
                break
            time.sleep(0.1)

        if self.current_position is not None:
            self.get_logger().info(f'Gripper position: {self.current_position:.4f}m')
        else:
            self.get_logger().warn('Gripper joint state not received yet')

        self.get_logger().info('Gripper Controller ready.')
        self.get_logger().info(f'  /gripper/open  → {self.open_pos}m')
        self.get_logger().info(f'  /gripper/close → {self.close_pos}m')

    def _joint_state_cb(self, msg: JointState):
        if self.joint_names[0] in msg.name:
            idx = msg.name.index(self.joint_names[0])
            self.current_position = msg.position[idx]

    def _open_cb(self, request, response):
        self.get_logger().info(
            f'Opening gripper → {self.open_pos}m (current: {self.current_position:.4f}m)')
        success = self._move_to(self.open_pos)
        response.success = success
        response.message = f'Gripper {"opened" if success else "FAILED to open"}'
        return response

    def _close_cb(self, request, response):
        self.get_logger().info(
            f'Closing gripper → {self.close_pos}m (current: {self.current_position:.4f}m)')
        success = self._move_to(self.close_pos)
        response.success = success
        response.message = f'Gripper {"closed" if success else "FAILED to close"}'
        return response

    def _move_to(self, target: float) -> bool:
        """Publish trajectory and wait for gripper to reach target."""
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [target]  # left finger only, right follows via mimic
        point.time_from_start = Duration(
            sec=int(self.move_dur),
            nanosec=int((self.move_dur % 1) * 1e9)
        )
        traj.points = [point]

        self.traj_pub.publish(traj)
        self.get_logger().info(f'Trajectory published: {target:.4f}m, duration={self.move_dur}s')

        # Wait for gripper to reach target (poll joint state)
        max_wait = 60.0  # wall clock seconds (sim is slow)
        start = time.time()
        while time.time() - start < max_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_position is not None:
                error = abs(self.current_position - target)
                if error < self.tolerance:
                    self.get_logger().info(
                        f'Gripper reached target: {self.current_position:.4f}m '
                        f'(error={error:.4f}m)')
                    return True
            time.sleep(0.2)

        # Timeout — report final position
        if self.current_position is not None:
            error = abs(self.current_position - target)
            self.get_logger().warn(
                f'Gripper timeout after {max_wait}s. '
                f'pos={self.current_position:.4f}m, error={error:.4f}m')
        else:
            self.get_logger().error('No joint state received!')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
