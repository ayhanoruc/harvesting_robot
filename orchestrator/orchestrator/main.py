#!/usr/bin/env python3
"""
Harvest Orchestrator — Main state machine for cotton harvesting

Top-level controller that coordinates the full harvest cycle:
    IDLE → SCANNING → [per cluster: APPROACHING → HARVESTING → TRANSFERRING] → IDLE

Full flow (target behavior):
    1. SCANNING: Panoramic scan → vision ML detects cluster centers
    2. For each cluster:
       a. APPROACHING: Move toward cluster, continuous prediction en route
          - Find optimal pre-grasp view (closest point with max boll visibility)
          - Optional: mini left-right scan for full cluster coverage
       b. HARVESTING: Get individual boll 3D positions, pick each one
          - Per boll: pick → deposit in reservoir → return to pre-grasp
          - Re-scan between picks to update boll list (future)
       c. TRANSFERRING: (handled within harvest_executor per boll)
    3. All clusters done → HOME → IDLE

Services Provided:
    /orchestrator/start_harvest (std_srvs/Trigger)
        - Kick off the full harvest cycle
    /orchestrator/stop (std_srvs/Trigger)
        - Emergency stop / abort current cycle
    /orchestrator/status (→ publishes to /orchestrator/status topic)

Service Clients Used:
    /explorer/start_scan         — Panoramic scan
    /detection/run_at_position   — Run vision pipeline at current pose
    /go_to_named                 — Named arm targets (home, cluster_N)
    /go_to_pose                  — Arbitrary Cartesian arm target
    /harvest/pick_boll           — Pick-and-place single boll
    /gripper/open                — Open gripper
    /gripper/close               — Close gripper

Topics Published:
    /orchestrator/status (std_msgs/String) — Current state
    /orchestrator/progress (std_msgs/String) — Detailed progress info

Parameters:
    config_file — Path to environment_config.yaml
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from harvester_interfaces.srv import HarvestBoll, RunDetectionPipeline
from geometry_msgs.msg import Point
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

import yaml
import os
import math
import time
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory


# ─── State definitions ──────────────────────────────────────────

class State(Enum):
    IDLE = 'IDLE'
    SCANNING = 'SCANNING'
    APPROACHING = 'APPROACHING'
    HARVESTING = 'HARVESTING'
    RETURNING = 'RETURNING'
    ERROR = 'ERROR'


@dataclass
class BollTarget:
    """A single boll to be picked."""
    position: List[float]       # [x, y, z] world frame
    cluster_id: str
    picked: bool = False


@dataclass
class ClusterPlan:
    """Plan for harvesting one cluster."""
    cluster_id: str
    center: List[float]         # [x, y, z] cluster center
    pre_grasp_view: Optional[List[float]] = None   # Determined during approach
    bolls: List[BollTarget] = field(default_factory=list)
    completed: bool = False


# ─── Main orchestrator ──────────────────────────────────────────

class OrchestratorNode(Node):

    def __init__(self):
        super().__init__('orchestrator_node')

        self.cb = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────
        self.declare_parameter('config_file', '')
        self.declare_parameter('pre_grasp_offset', 0.15)

        self.pre_grasp_offset = self.get_parameter('pre_grasp_offset').value

        # ── State ───────────────────────────────────────────
        self.state = State.IDLE
        self.cluster_plans: List[ClusterPlan] = []
        self.current_cluster_idx = 0

        # ── Config ──────────────────────────────────────────
        self.config = self._load_config()

        # ── Publishers ──────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/orchestrator/status', 10)
        self.progress_pub = self.create_publisher(String, '/orchestrator/progress', 10)

        # ── Service clients ─────────────────────────────────
        self.scan_cli = self.create_client(
            Trigger, '/explorer/start_scan', callback_group=self.cb)
        self.detect_cli = self.create_client(
            RunDetectionPipeline, '/detection/run_at_position', callback_group=self.cb)
        self.go_to_named_cli = self.create_client(
            SetBool, '/go_to_named', callback_group=self.cb)
        self.go_to_pose_cli = self.create_client(
            SetBool, '/go_to_pose', callback_group=self.cb)
        self.pick_boll_cli = self.create_client(
            HarvestBoll, '/harvest/pick_boll', callback_group=self.cb)
        self.gripper_open_cli = self.create_client(
            Trigger, '/gripper/open', callback_group=self.cb)
        self.arm_set_params_cli = self.create_client(
            SetParameters, '/arm_commander/set_parameters', callback_group=self.cb)

        # ── Services provided ───────────────────────────────
        self.create_service(
            Trigger, '/orchestrator/start_harvest', self._start_harvest_cb,
            callback_group=self.cb)
        self.create_service(
            Trigger, '/orchestrator/stop', self._stop_cb,
            callback_group=self.cb)

        self._set_state(State.IDLE)
        self.get_logger().info('='*50)
        self.get_logger().info('HARVEST ORCHESTRATOR ready')
        self.get_logger().info('  Call /orchestrator/start_harvest to begin')
        self.get_logger().info('='*50)

    # ─── Config loading ─────────────────────────────────────────

    def _load_config(self) -> dict:
        config_file = self.get_parameter('config_file').value
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                return yaml.safe_load(f)
        self.get_logger().warn('No config file specified')
        return {}

    # ─── State management ───────────────────────────────────────

    def _set_state(self, new_state: State):
        old = self.state
        self.state = new_state
        self.status_pub.publish(String(data=new_state.value))
        self.get_logger().info(f'STATE: {old.value} → {new_state.value}')

    def _publish_progress(self, msg: str):
        self.progress_pub.publish(String(data=msg))
        self.get_logger().info(f'PROGRESS: {msg}')

    # ─── Service callbacks ──────────────────────────────────────

    def _start_harvest_cb(self, request, response):
        """Entry point: kick off the full harvest cycle."""
        if self.state != State.IDLE:
            response.success = False
            response.message = f'Cannot start: currently in {self.state.value}'
            return response

        self.get_logger().info('='*50)
        self.get_logger().info('HARVEST CYCLE STARTING')
        self.get_logger().info('='*50)

        try:
            self._run_harvest_cycle()
            response.success = True
            response.message = 'Harvest cycle completed'
        except Exception as e:
            self.get_logger().error(f'Harvest cycle failed: {e}')
            self._set_state(State.ERROR)
            response.success = False
            response.message = str(e)

        return response

    def _stop_cb(self, request, response):
        """Emergency stop."""
        self.get_logger().warn('STOP requested')
        self._set_state(State.IDLE)
        response.success = True
        response.message = 'Stopped'
        return response

    # ─── Main harvest cycle ─────────────────────────────────────

    def _run_harvest_cycle(self):
        """
        Full harvest cycle:
            1. SCANNING   — scan field, find clusters
            2. Per cluster:
               a. APPROACHING  — move toward cluster (+ vision)
               b. HARVESTING   — pick each boll
            3. RETURNING  — go home
        """
        # ── Phase 1: SCANNING ──────────────────────────────
        self._set_state(State.SCANNING)
        cluster_positions = self._phase_scanning()

        if not cluster_positions:
            self._publish_progress('No clusters found, returning home')
            self._go_home()
            self._set_state(State.IDLE)
            return

        # ── Build cluster plans ────────────────────────────
        self.cluster_plans = []
        for cid, pos in cluster_positions.items():
            plan = ClusterPlan(cluster_id=cid, center=pos)
            self.cluster_plans.append(plan)
        self._publish_progress(
            f'Found {len(self.cluster_plans)} clusters: '
            f'{[p.cluster_id for p in self.cluster_plans]}')

        # ── Phase 2: Per-cluster harvest ───────────────────
        for idx, plan in enumerate(self.cluster_plans):
            self.current_cluster_idx = idx
            self._publish_progress(
                f'Cluster {idx+1}/{len(self.cluster_plans)}: {plan.cluster_id}')

            # 2a. APPROACHING — move toward cluster
            self._set_state(State.APPROACHING)
            self._phase_approaching(plan)

            # 2b. HARVESTING — pick all bolls in cluster
            self._set_state(State.HARVESTING)
            self._phase_harvesting(plan)

            plan.completed = True

        # ── Phase 3: RETURNING ─────────────────────────────
        self._set_state(State.RETURNING)
        self._go_home()

        # ── Done ───────────────────────────────────────────
        picked = sum(1 for p in self.cluster_plans
                     for b in p.bolls if b.picked)
        total = sum(len(p.bolls) for p in self.cluster_plans)
        self._publish_progress(f'HARVEST COMPLETE: {picked}/{total} bolls picked')
        self._set_state(State.IDLE)

    # ─── Phase: SCANNING ────────────────────────────────────────

    def _phase_scanning(self) -> dict:
        """
        Scan the field and return cluster positions.

        Target behavior:
            1. Run panoramic scan (explorer)
            2. Run detection pipeline at each viewpoint
            3. Collect and merge cluster 3D positions

        Sim placeholder:
            Return cluster positions from config file directly.
            (No actual scan — positions are ground truth.)
        """
        self._publish_progress('Scanning field for clusters...')

        # ── PLACEHOLDER: Use config positions as ground truth ──
        clusters = self.config.get('clusters', {})
        result = {}
        for cid, cdata in clusters.items():
            pos = cdata.get('position', [0, 0, 0])
            result[cid] = pos
            self.get_logger().info(f'  Cluster {cid}: {pos}')

        # ── TODO (real implementation): ──
        # 1. self._go_home()
        # 2. Call /explorer/start_scan → panoramic sweep
        # 3. At each viewpoint, call /detection/run_at_position
        # 4. Merge detections, return cluster centers
        # Example:
        #   scan_result = self._call_trigger(self.scan_cli)
        #   for viewpoint in scan_viewpoints:
        #       detections = self._call_detection()
        #       merge into result...

        return result

    # ─── Phase: APPROACHING ─────────────────────────────────────

    def _phase_approaching(self, plan: ClusterPlan):
        """
        Approach a cluster and determine pre-grasp view + boll positions.

        Target behavior:
            1. Move from current position toward cluster center
            2. While moving, run continuous vision predictions
            3. At each intermediate position, count visible bolls
            4. Stop at the position with maximum boll visibility
               → this becomes the "pre-grasp view"
            5. Optional: mini left-right scan for full coverage
            6. Get 3D positions of all visible bolls

        Sim placeholder:
            - Compute pre-grasp view as fixed offset from cluster center
            - Use config boll position as the single boll target
            - Move arm to pre-grasp view
        """
        cx, cy, cz = plan.center
        self._publish_progress(
            f'Approaching {plan.cluster_id} at ({cx:.2f}, {cy:.2f}, {cz:.2f})')

        # ── PLACEHOLDER: Fixed pre-grasp computation ───────
        pre_x, pre_y, pre_z = self._compute_pre_grasp(cx, cy, cz)
        plan.pre_grasp_view = [pre_x, pre_y, pre_z]

        # Move to pre-grasp view
        self._go_to_xyz(pre_x, pre_y, pre_z)

        # Use config position as the single boll target
        plan.bolls = [
            BollTarget(
                position=plan.center.copy(),
                cluster_id=plan.cluster_id)
        ]
        self._publish_progress(
            f'Pre-grasp view: ({pre_x:.2f}, {pre_y:.2f}, {pre_z:.2f}), '
            f'{len(plan.bolls)} boll(s) to pick')

        # ── TODO (real implementation): ──
        # 1. Compute intermediate waypoints from current pos → cluster
        # 2. At each waypoint:
        #    a. Move arm there
        #    b. Run /detection/run_at_position
        #    c. Count bolls, track best viewpoint
        # 3. Optimal viewpoint → plan.pre_grasp_view
        # 4. Run final detection → plan.bolls = detected boll positions
        # 5. Optional: mini scan (rotate ±15° around cluster axis)
        #
        # Example pseudocode:
        #   best_count = 0
        #   for t in [0.3, 0.5, 0.7, 0.85, 1.0]:  # approach fractions
        #       wp = interpolate(current_pos, cluster_center, t)
        #       self._go_to_xyz(*wp)
        #       detections = self._call_detection()
        #       if len(detections) > best_count:
        #           best_count = len(detections)
        #           plan.pre_grasp_view = wp
        #   self._go_to_xyz(*plan.pre_grasp_view)
        #   final_detections = self._call_detection()
        #   plan.bolls = [BollTarget(d.position, ...) for d in final_detections]

    # ─── Phase: HARVESTING ──────────────────────────────────────

    def _phase_harvesting(self, plan: ClusterPlan):
        """
        Pick all bolls in a cluster, one by one.

        For each boll:
            1. Call /harvest/pick_boll with boll position + pre-grasp view
            2. Executor handles: pre-grasp → open → approach → close → lift
                                 → reservoir → release → return to pre-grasp
            3. Mark boll as picked
            4. (Future: re-scan to update remaining boll positions)

        After all bolls: cluster is complete.
        """
        if not plan.bolls:
            self._publish_progress(f'{plan.cluster_id}: no bolls to pick')
            return

        self._publish_progress(
            f'{plan.cluster_id}: picking {len(plan.bolls)} boll(s)')

        for i, boll in enumerate(plan.bolls):
            if boll.picked:
                continue

            self._publish_progress(
                f'{plan.cluster_id}: boll {i+1}/{len(plan.bolls)} '
                f'at ({boll.position[0]:.2f}, {boll.position[1]:.2f}, '
                f'{boll.position[2]:.2f})')

            # Call harvest executor
            success = self._pick_single_boll(boll, plan.pre_grasp_view)
            boll.picked = success

            if success:
                self._publish_progress(f'  Boll {i+1} picked successfully')
            else:
                self._publish_progress(f'  Boll {i+1} FAILED, skipping')
                # TODO: retry logic? re-scan? skip to next?

            # ── TODO (real implementation): ──
            # After each pick, optionally re-scan from pre-grasp view
            # to update boll positions (bolls may have shifted):
            #   self._go_to_xyz(*plan.pre_grasp_view)
            #   new_detections = self._call_detection()
            #   update plan.bolls with new positions...

        picked = sum(1 for b in plan.bolls if b.picked)
        self._publish_progress(
            f'{plan.cluster_id}: DONE ({picked}/{len(plan.bolls)} picked)')

    # ─── Arm movement helpers ───────────────────────────────────

    def _go_to_xyz(self, x, y, z) -> bool:
        """Set arm_commander params and call /go_to_pose."""
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
        ]
        req = SetParameters.Request(parameters=params)

        if not self.arm_set_params_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('arm_commander set_parameters not available')
            return False
        future = self.arm_set_params_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not self.go_to_pose_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/go_to_pose not available')
            return False
        go_req = SetBool.Request(data=True)
        future = self.go_to_pose_cli.call_async(go_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=120.0)
        if future.result() is not None:
            return future.result().success
        return False

    def _go_home(self) -> bool:
        """Go to home position via /go_to_named."""
        params = [
            Parameter(name='target_name',
                      value=ParameterValue(
                          type=ParameterType.PARAMETER_STRING,
                          string_value='home')),
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

    def _pick_single_boll(self, boll: BollTarget,
                          pre_grasp_view: List[float]) -> bool:
        """Call /harvest/pick_boll service."""
        if not self.pick_boll_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/harvest/pick_boll not available')
            return False

        req = HarvestBoll.Request()
        req.boll_position = Point(
            x=boll.position[0], y=boll.position[1], z=boll.position[2])
        req.pre_grasp_position = Point(
            x=pre_grasp_view[0], y=pre_grasp_view[1], z=pre_grasp_view[2])

        future = self.pick_boll_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=600.0)

        if future.result() is not None:
            return future.result().success
        return False

    # ─── Geometry helpers ───────────────────────────────────────

    def _compute_pre_grasp(self, boll_x, boll_y, boll_z):
        """Pre-grasp = offset back from boll along approach vector."""
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

    # ─── Detection helpers (for future use) ─────────────────────

    def _call_detection(self):
        """
        Call /detection/run_at_position and return detected bolls.

        TODO: Implement when connecting vision pipeline.
        Returns list of BollTarget from detection results.
        """
        # req = RunDetectionPipeline.Request()
        # req.focus_iterations = 2
        # future = self.detect_cli.call_async(req)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        # result = future.result()
        # return [BollTarget(position=[d.position.x, d.position.y, d.position.z],
        #                    cluster_id=d.cluster_id)
        #         for d in result.detections]
        return []

    def _call_trigger(self, client) -> bool:
        """Generic trigger service call."""
        if not client.wait_for_service(timeout_sec=5.0):
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=120.0)
        if future.result() is not None:
            return future.result().success
        return False


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
