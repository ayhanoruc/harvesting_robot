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

Service Clients Used:
    /go_to_pose   (std_srvs/SetBool)  — arm_commander
    /go_to_named  (std_srvs/SetBool)  — arm_commander
    /gripper/open  (std_srvs/Trigger)
    /gripper/close (std_srvs/Trigger)

Parameters:
    pre_grasp_offset  — distance back from boll along approach (default: 0.15m)
    lift_height       — how high to lift after grasp (default: 0.15m)
    config_file       — path to environment_config.yaml (for reservoir position)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger, SetBool
from harvester_interfaces.srv import HarvestBoll
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

import math
import time
import yaml
import os


class HarvestExecutor(Node):

    def __init__(self):
        super().__init__('harvest_executor')

        self.cb = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('pre_grasp_offset', 0.08)
        self.declare_parameter('lift_height', 0.15)
        self.declare_parameter('config_file', '')

        self.pre_grasp_offset = self.get_parameter('pre_grasp_offset').value
        self.lift_height = self.get_parameter('lift_height').value

        # Load reservoir position from config
        self.reservoir_pos = self._load_reservoir_position()

        # Track pick stats
        self.total_picks = 0
        self.successful_picks = 0

        # ── Service clients ─────────────────────────────────────
        self.go_to_pose_cli = self.create_client(
            SetBool, '/go_to_pose', callback_group=self.cb)
        self.go_to_named_cli = self.create_client(
            SetBool, '/go_to_named', callback_group=self.cb)
        self.gripper_open_cli = self.create_client(
            Trigger, '/gripper/open', callback_group=self.cb)
        self.gripper_close_cli = self.create_client(
            Trigger, '/gripper/close', callback_group=self.cb)
        self.arm_set_params_cli = self.create_client(
            SetParameters, '/arm_commander/set_parameters', callback_group=self.cb)

        # ── Service provided ────────────────────────────────────
        self.create_service(
            HarvestBoll, '/harvest/pick_boll', self._pick_boll_cb,
            callback_group=self.cb)

        self.get_logger().info('=' * 50)
        self.get_logger().info('HARVEST EXECUTOR ready')
        self.get_logger().info(f'  pre_grasp_offset: {self.pre_grasp_offset}m')
        self.get_logger().info(f'  lift_height:      {self.lift_height}m')
        self.get_logger().info(f'  reservoir:        {self.reservoir_pos}')
        self.get_logger().info(f'  service:          /harvest/pick_boll')
        self.get_logger().info('=' * 50)

    # ─── Config ─────────────────────────────────────────────────

    def _load_reservoir_position(self):
        config_file = self.get_parameter('config_file').value
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            res = config.get('landmarks', {}).get('reservoir', {})
            pos = res.get('position', [0.0, 0.6, 0.3])
            self.get_logger().info(f'Reservoir from config: {pos}')
            return pos
        self.get_logger().warn('No config file, using default reservoir [0.0, 0.6, 0.3]')
        return [0.0, 0.6, 0.3]

    def _wait_future(self, future, timeout_sec=30.0):
        """Poll-wait for a service future (safe with MultiThreadedExecutor)."""
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout_sec:
                self.get_logger().warn(
                    f'Service call timed out after {timeout_sec}s')
                return None
            time.sleep(0.05)
        return future.result()

    # ─── Main pick sequence ─────────────────────────────────────

    def _pick_boll_cb(self, request, response):
        """Full pick-and-place for a single boll."""
        boll = request.boll_position
        pre_grasp = request.pre_grasp_position
        self.total_picks += 1

        self.get_logger().info('=' * 50)
        self.get_logger().info(
            f'PICK #{self.total_picks} START: '
            f'boll=({boll.x:.3f}, {boll.y:.3f}, {boll.z:.3f})')
        self.get_logger().info(
            f'  pre_grasp=({pre_grasp.x:.3f}, {pre_grasp.y:.3f}, '
            f'{pre_grasp.z:.3f})')
        self.get_logger().info(
            f'  reservoir=({self.reservoir_pos[0]:.3f}, '
            f'{self.reservoir_pos[1]:.3f}, {self.reservoir_pos[2]:.3f})')
        self.get_logger().info(
            f'  lift_height={self.lift_height}m')

        t_start = time.time()

        try:
            # Step 1: Go to pre-grasp position (face cluster)
            self.get_logger().info(
                f'[1/8] PRE-GRASP: going to '
                f'({pre_grasp.x:.3f}, {pre_grasp.y:.3f}, {pre_grasp.z:.3f})')
            t_step = time.time()
            if not self._go_to_xyz(pre_grasp.x, pre_grasp.y, pre_grasp.z,
                                   approach_orientation=True):
                raise RuntimeError('Failed to reach pre-grasp')
            self.get_logger().info(
                f'[1/8] PRE-GRASP: reached in {time.time()-t_step:.1f}s')

            # Step 2: Open gripper
            self.get_logger().info('[2/8] GRIPPER OPEN: opening before approach')
            t_step = time.time()
            if not self._call_gripper('open'):
                raise RuntimeError('Failed to open gripper')
            self.get_logger().info(
                f'[2/8] GRIPPER OPEN: done in {time.time()-t_step:.1f}s')

            # Step 3: Go to grasp position (3cm before boll center to avoid collision)
            standoff = 0.03
            dx, dy = boll.x, boll.y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > 0.01:
                gx = boll.x - standoff * (dx / dist)
                gy = boll.y - standoff * (dy / dist)
            else:
                gx, gy = boll.x, boll.y
            gz = boll.z
            self.get_logger().info(
                f'[3/8] APPROACH BOLL: going to '
                f'({gx:.3f}, {gy:.3f}, {gz:.3f}) '
                f'(3cm standoff from boll center)')
            t_step = time.time()
            if not self._go_to_xyz(gx, gy, gz,
                                   approach_orientation=True):
                raise RuntimeError('Failed to reach boll')
            self.get_logger().info(
                f'[3/8] APPROACH BOLL: reached in {time.time()-t_step:.1f}s')

            # Step 4: Close gripper
            self.get_logger().info('[4/8] GRIPPER CLOSE: grasping boll')
            t_step = time.time()
            if not self._call_gripper('close'):
                raise RuntimeError('Failed to close gripper')
            self.get_logger().info(
                f'[4/8] GRIPPER CLOSE: done in {time.time()-t_step:.1f}s')

            # Step 5: Retract to pre-grasp (safer than lifting in place at workspace edge)
            self.get_logger().info(
                f'[5/8] RETRACT: going to pre-grasp '
                f'({pre_grasp.x:.3f}, {pre_grasp.y:.3f}, {pre_grasp.z:.3f})')
            t_step = time.time()
            if not self._go_to_xyz(pre_grasp.x, pre_grasp.y, pre_grasp.z,
                                   approach_orientation=True):
                raise RuntimeError('Failed to retract to pre-grasp')
            self.get_logger().info(
                f'[5/8] RETRACT: reached in {time.time()-t_step:.1f}s')

            # Step 6: Go to reservoir (hover 15cm above box top to avoid collision)
            rx, ry, rz = self.reservoir_pos
            rz_hover = rz + 0.15
            self.get_logger().info(
                f'[6/8] RESERVOIR: going to ({rx:.3f}, {ry:.3f}, {rz_hover:.3f})'
                f' (config z={rz:.2f} + 0.15m hover)')
            t_step = time.time()
            if not self._go_to_xyz(rx, ry, rz_hover,
                                   approach_orientation=True, use_direct=True):
                raise RuntimeError('Failed to reach reservoir')
            self.get_logger().info(
                f'[6/8] RESERVOIR: reached in {time.time()-t_step:.1f}s')

            # Step 7: Open gripper (release boll)
            self.get_logger().info('[7/8] RELEASE: opening gripper to drop boll')
            t_step = time.time()
            if not self._call_gripper('open'):
                raise RuntimeError('Failed to release')
            self.get_logger().info(
                f'[7/8] RELEASE: done in {time.time()-t_step:.1f}s')

            # Step 8: Return to pre-grasp viewpoint (face cluster again)
            self.get_logger().info(
                f'[8/8] RETURN: going back to pre-grasp '
                f'({pre_grasp.x:.3f}, {pre_grasp.y:.3f}, {pre_grasp.z:.3f})')
            t_step = time.time()
            if not self._go_to_xyz(pre_grasp.x, pre_grasp.y, pre_grasp.z,
                                   approach_orientation=True):
                raise RuntimeError('Failed to return to pre-grasp')
            self.get_logger().info(
                f'[8/8] RETURN: reached in {time.time()-t_step:.1f}s')

            elapsed = time.time() - t_start
            self.successful_picks += 1
            self.get_logger().info(
                f'PICK #{self.total_picks} SUCCESS: '
                f'{elapsed:.1f}s total, '
                f'{self.successful_picks}/{self.total_picks} lifetime')
            self.get_logger().info('=' * 50)
            response.success = True
            response.message = f'Boll picked in {elapsed:.1f}s'

        except RuntimeError as e:
            elapsed = time.time() - t_start
            self.get_logger().error(
                f'PICK #{self.total_picks} FAILED at {elapsed:.1f}s: {e}')
            self.get_logger().info('=' * 50)
            response.success = False
            response.message = str(e)

        return response

    # ─── Arm movement helpers ───────────────────────────────────

    def _go_to_xyz(self, x, y, z, approach_orientation=False,
                   use_direct=False) -> bool:
        """Set arm_commander params and call /go_to_pose."""
        self.get_logger().info(
            f'[ARM] Setting target: ({x:.3f}, {y:.3f}, {z:.3f})'
            f' approach={approach_orientation} direct={use_direct}')

        params = [
            Parameter(name='target_x',
                      value=ParameterValue(
                          type=ParameterType.PARAMETER_DOUBLE, double_value=x)),
            Parameter(name='target_y',
                      value=ParameterValue(
                          type=ParameterType.PARAMETER_DOUBLE, double_value=y)),
            Parameter(name='target_z',
                      value=ParameterValue(
                          type=ParameterType.PARAMETER_DOUBLE, double_value=z)),
            Parameter(name='use_approach_orientation',
                      value=ParameterValue(
                          type=ParameterType.PARAMETER_BOOL,
                          bool_value=approach_orientation)),
            Parameter(name='use_direct_trajectory',
                      value=ParameterValue(
                          type=ParameterType.PARAMETER_BOOL,
                          bool_value=use_direct)),
        ]
        req = SetParameters.Request(parameters=params)

        if not self.arm_set_params_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '[ARM] arm_commander/set_parameters not available')
            return False
        future = self.arm_set_params_cli.call_async(req)
        self._wait_future(future,5.0)
        if future.result() is None:
            self.get_logger().error('[ARM] set_parameters returned None')
            return False

        # Call go_to_pose
        if not self.go_to_pose_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('[ARM] /go_to_pose not available')
            return False
        go_req = SetBool.Request(data=True)
        future = self.go_to_pose_cli.call_async(go_req)
        self._wait_future(future,120.0)

        if future.result() is not None:
            ok = future.result().success
            msg = future.result().message
            if ok:
                self.get_logger().info(
                    f'[ARM] Reached ({x:.3f}, {y:.3f}, {z:.3f}) - {msg}')
            else:
                self.get_logger().error(
                    f'[ARM] Failed to reach ({x:.3f}, {y:.3f}, {z:.3f}) - {msg}')
            return ok

        self.get_logger().error('[ARM] go_to_pose timeout (120s)')
        return False

    def _go_to_named(self, name: str) -> bool:
        """Set target_name param and call /go_to_named."""
        self.get_logger().info(f'[ARM] Going to named target: {name}')

        params = [
            Parameter(name='target_name',
                      value=ParameterValue(
                          type=ParameterType.PARAMETER_STRING,
                          string_value=name)),
        ]
        req = SetParameters.Request(parameters=params)
        if not self.arm_set_params_cli.wait_for_service(timeout_sec=5.0):
            return False
        future = self.arm_set_params_cli.call_async(req)
        self._wait_future(future,5.0)

        if not self.go_to_named_cli.wait_for_service(timeout_sec=5.0):
            return False
        go_req = SetBool.Request(data=True)
        future = self.go_to_named_cli.call_async(go_req)
        self._wait_future(future,120.0)

        if future.result() is not None:
            ok = future.result().success
            self.get_logger().info(
                f'[ARM] go_to_named({name}): {"OK" if ok else "FAIL"}')
            return ok
        return False

    # ─── Gripper helpers ────────────────────────────────────────

    def _call_gripper(self, action: str) -> bool:
        """Call gripper open or close service."""
        # BYPASS: gripper disabled for demo — always succeed
        self.get_logger().info(f'[GRIPPER] {action}: BYPASSED (demo mode)')
        return True

        # --- original implementation (re-enable when gripper works) ---
        # cli = self.gripper_open_cli if action == 'open' else self.gripper_close_cli
        # if not cli.wait_for_service(timeout_sec=5.0):
        #     self.get_logger().error(f'[GRIPPER] /gripper/{action} not available')
        #     return False
        #
        # future = cli.call_async(Trigger.Request())
        # self._wait_future(future,120.0)
        #
        # if future.result() is not None:
        #     ok = future.result().success
        #     msg = future.result().message
        #     self.get_logger().info(
        #         f'[GRIPPER] {action}: {"OK" if ok else "FAIL"} - {msg}')
        #     return ok
        #
        # self.get_logger().error(f'[GRIPPER] {action} timeout (120s)')
        # return False

    # ─── Pre-grasp computation ──────────────────────────────────

    def compute_pre_grasp(self, boll_x, boll_y, boll_z):
        """Pre-grasp = offset back from boll along horizontal approach."""
        dx, dy = boll_x, boll_y
        length = math.sqrt(dx * dx + dy * dy)
        if length < 0.01:
            dx, dy = 1.0, 0.0
            length = 1.0

        offset = self.pre_grasp_offset
        pre_x = boll_x - offset * (dx / length)
        pre_y = boll_y - offset * (dy / length)
        pre_z = boll_z

        return pre_x, pre_y, pre_z


def main(args=None):
    rclpy.init(args=args)
    node = HarvestExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
