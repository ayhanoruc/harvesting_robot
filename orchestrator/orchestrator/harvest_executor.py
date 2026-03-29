#!/usr/bin/env python3
"""
Harvest Executor — Single-boll pick-and-place coordinator

Mid-level node that orchestrates arm_commander + gripper_controller
to pick one cotton boll and deposit it in the reservoir.

Pick sequence for one boll:
    1. Go to pre-grasp position (offset back from boll along approach vector)
    2. Open gripper
    3. Go to grasp position (boll center)
    4. Close gripper
    5. Lift (z + lift_height)
    6. Go to reservoir
    7. Open gripper (release)
    8. Return to pre-grasp viewpoint

Services Provided:
    /harvest/pick_boll (harvester_interfaces/srv/HarvestBoll)
        - Pick a single boll given its 3D position + pre-grasp viewpoint

Service Clients Used:
    /go_to_pose   (std_srvs/SetBool)  — arm_commander, target via params
    /go_to_named  (std_srvs/SetBool)  — arm_commander, named targets
    /gripper/open  (std_srvs/Trigger)
    /gripper/close (std_srvs/Trigger)

Parameters:
    pre_grasp_offset  — distance back from boll along approach (default: 0.15m)
    lift_height       — how high to lift after grasp (default: 0.15m)
    reservoir_x/y/z   — reservoir drop position (from config)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger, SetBool
from harvester_interfaces.srv import HarvestBoll
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

import math
import time
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class HarvestExecutor(Node):

    def __init__(self):
        super().__init__('harvest_executor')

        self.cb = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('pre_grasp_offset', 0.15)
        self.declare_parameter('lift_height', 0.15)
        self.declare_parameter('config_file', '')

        self.pre_grasp_offset = self.get_parameter('pre_grasp_offset').value
        self.lift_height = self.get_parameter('lift_height').value

        # Load reservoir position from config
        self.reservoir_pos = self._load_reservoir_position()

        # ── Service clients ─────────────────────────────────────
        self.go_to_pose_cli = self.create_client(
            SetBool, '/go_to_pose', callback_group=self.cb)
        self.go_to_named_cli = self.create_client(
            SetBool, '/go_to_named', callback_group=self.cb)
        self.gripper_open_cli = self.create_client(
            Trigger, '/gripper/open', callback_group=self.cb)
        self.gripper_close_cli = self.create_client(
            Trigger, '/gripper/close', callback_group=self.cb)
        # For setting arm_commander parameters before go_to_pose
        self.arm_set_params_cli = self.create_client(
            SetParameters, '/arm_commander/set_parameters', callback_group=self.cb)

        # ── Service provided ────────────────────────────────────
        self.create_service(
            HarvestBoll, '/harvest/pick_boll', self._pick_boll_cb,
            callback_group=self.cb)

        self.get_logger().info('Harvest Executor ready.')
        self.get_logger().info(f'  pre_grasp_offset: {self.pre_grasp_offset}m')
        self.get_logger().info(f'  lift_height: {self.lift_height}m')
        self.get_logger().info(f'  reservoir: {self.reservoir_pos}')

    # ─── Config ─────────────────────────────────────────────────

    def _load_reservoir_position(self):
        """Load reservoir position from environment_config.yaml."""
        config_file = self.get_parameter('config_file').value
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            res = config.get('landmarks', {}).get('reservoir', {})
            pos = res.get('position', [0.0, 0.6, 0.3])
            self.get_logger().info(f'Reservoir from config: {pos}')
            return pos
        self.get_logger().warn('No config file, using default reservoir position')
        return [0.0, 0.6, 0.3]

    # ─── Main pick sequence ─────────────────────────────────────

    def _pick_boll_cb(self, request, response):
        """Full pick-and-place for a single boll."""
        boll = request.boll_position
        pre_grasp = request.pre_grasp_position
        self.get_logger().info(
            f'PICK BOLL: ({boll.x:.3f}, {boll.y:.3f}, {boll.z:.3f})')

        try:
            # Step 1: Go to pre-grasp position
            self.get_logger().info('[1/8] Going to pre-grasp...')
            if not self._go_to_xyz(pre_grasp.x, pre_grasp.y, pre_grasp.z):
                raise RuntimeError('Failed to reach pre-grasp')

            # Step 2: Open gripper
            self.get_logger().info('[2/8] Opening gripper...')
            if not self._call_gripper('open'):
                raise RuntimeError('Failed to open gripper')

            # Step 3: Go to grasp position (boll center)
            self.get_logger().info('[3/8] Approaching boll...')
            if not self._go_to_xyz(boll.x, boll.y, boll.z):
                raise RuntimeError('Failed to reach boll')

            # Step 4: Close gripper
            self.get_logger().info('[4/8] Closing gripper...')
            if not self._call_gripper('close'):
                raise RuntimeError('Failed to close gripper')

            # Step 5: Lift
            lift_z = boll.z + self.lift_height
            self.get_logger().info(f'[5/8] Lifting to z={lift_z:.3f}...')
            if not self._go_to_xyz(boll.x, boll.y, lift_z):
                raise RuntimeError('Failed to lift')

            # Step 6: Go to reservoir
            rx, ry, rz = self.reservoir_pos
            self.get_logger().info(f'[6/8] Going to reservoir ({rx}, {ry}, {rz})...')
            if not self._go_to_xyz(rx, ry, rz):
                raise RuntimeError('Failed to reach reservoir')

            # Step 7: Open gripper (release boll)
            self.get_logger().info('[7/8] Releasing boll...')
            if not self._call_gripper('open'):
                raise RuntimeError('Failed to release')

            # Step 8: Return to pre-grasp viewpoint
            self.get_logger().info('[8/8] Returning to pre-grasp view...')
            if not self._go_to_xyz(pre_grasp.x, pre_grasp.y, pre_grasp.z):
                raise RuntimeError('Failed to return to pre-grasp')

            self.get_logger().info('PICK BOLL: SUCCESS')
            response.success = True
            response.message = 'Boll picked and deposited'

        except RuntimeError as e:
            self.get_logger().error(f'PICK BOLL FAILED: {e}')
            response.success = False
            response.message = str(e)

        return response

    # ─── Arm movement helpers ───────────────────────────────────

    def _go_to_xyz(self, x, y, z) -> bool:
        """Set arm_commander params and call /go_to_pose."""
        # Set target parameters on arm_commander
        params = [
            Parameter(
                name='target_x',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=x)),
            Parameter(
                name='target_y',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=y)),
            Parameter(
                name='target_z',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=z)),
        ]
        req = SetParameters.Request(parameters=params)

        if not self.arm_set_params_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('arm_commander set_parameters not available')
            return False
        future = self.arm_set_params_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        # Call go_to_pose
        if not self.go_to_pose_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/go_to_pose not available')
            return False
        go_req = SetBool.Request(data=True)
        future = self.go_to_pose_cli.call_async(go_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=120.0)

        if future.result() is not None:
            return future.result().success
        return False

    def _go_to_named(self, name: str) -> bool:
        """Set target_name param and call /go_to_named."""
        params = [
            Parameter(
                name='target_name',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=name)),
        ]
        req = SetParameters.Request(parameters=params)
        if not self.arm_set_params_cli.wait_for_service(timeout_sec=5.0):
            return False
        future = self.arm_set_params_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not self.go_to_named_cli.wait_for_service(timeout_sec=5.0):
            return False
        go_req = SetBool.Request(data=True)
        future = self.go_to_named_cli.call_async(go_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=120.0)

        if future.result() is not None:
            return future.result().success
        return False

    # ─── Gripper helpers ────────────────────────────────────────

    def _call_gripper(self, action: str) -> bool:
        """Call gripper open or close service."""
        cli = self.gripper_open_cli if action == 'open' else self.gripper_close_cli
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'/gripper/{action} not available')
            return False
        future = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=120.0)
        if future.result() is not None:
            return future.result().success
        return False

    # ─── Pre-grasp computation ──────────────────────────────────

    def compute_pre_grasp(self, boll_x, boll_y, boll_z):
        """
        Compute pre-grasp position: offset back from boll along
        horizontal approach vector (from robot base to boll).

        Returns (x, y, z) of pre-grasp position.
        """
        # Approach direction: from base (0,0) toward boll (horizontal)
        dx, dy = boll_x, boll_y
        length = math.sqrt(dx * dx + dy * dy)
        if length < 0.01:
            # Boll directly above base — approach from +X
            dx, dy = 1.0, 0.0
            length = 1.0

        offset = self.pre_grasp_offset
        pre_x = boll_x - offset * (dx / length)
        pre_y = boll_y - offset * (dy / length)
        pre_z = boll_z  # same height

        return pre_x, pre_y, pre_z


def main(args=None):
    rclpy.init(args=args)
    node = HarvestExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
