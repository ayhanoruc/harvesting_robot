#!/usr/bin/env python3
"""
Harvest Orchestrator — Main state machine for cotton harvesting

Top-level controller that coordinates the full harvest cycle:
    IDLE -> SCANNING -> [per cluster: APPROACHING -> HARVESTING] -> RETURNING -> IDLE

Full flow:
    1. SCANNING: Panoramic scan (3 positions, j1 rotate from HOME)
       -> YOLO detects clusters -> depth -> 3D positions -> merge
    2. Per cluster:
       a. APPROACHING: Go to pre-grasp cluster view
          -> Run YOLO detect (individual bolls) + depth -> boll 3D positions
       b. HARVESTING: Pick each boll via harvest_executor
          -> pre-grasp -> open -> approach -> close -> lift -> reservoir -> open -> return
    3. RETURNING: Go HOME

Services Provided:
    /orchestrator/start_harvest (std_srvs/Trigger)
    /orchestrator/stop (std_srvs/Trigger)

Topics Published:
    /orchestrator/status (std_msgs/String)
    /orchestrator/progress (std_msgs/String)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Point
from harvester_interfaces.srv import (
    HarvestBoll, GetDetectedClusters, YoloDetect, PixelTo3D)
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

import yaml
import os
import math
import time
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Optional


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
    confidence: float = 0.0
    picked: bool = False


@dataclass
class ClusterPlan:
    """Plan for harvesting one cluster."""
    cluster_id: str
    center: List[float]         # [x, y, z] cluster center
    pre_grasp_view: Optional[List[float]] = None
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
        self.declare_parameter('scan_timeout', 600.0)     # wall-clock seconds
        self.declare_parameter('camera_settle_time', 3.0)  # seconds after move
        self.declare_parameter('use_vision_for_bolls', True)
        self.declare_parameter('use_vision_for_scan', True)

        self.pre_grasp_offset = self.get_parameter('pre_grasp_offset').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.camera_settle_time = self.get_parameter('camera_settle_time').value
        self.use_vision_for_bolls = self.get_parameter('use_vision_for_bolls').value
        self.use_vision_for_scan = self.get_parameter('use_vision_for_scan').value

        # ── State ───────────────────────────────────────────
        self.state = State.IDLE
        self.cluster_plans: List[ClusterPlan] = []
        self.current_cluster_idx = 0
        self._scan_complete = False
        self._stop_requested = False

        # ── Config ──────────────────────────────────────────
        self.config = self._load_config()

        # ── Publishers ──────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/orchestrator/status', 10)
        self.progress_pub = self.create_publisher(String, '/orchestrator/progress', 10)

        # ── Subscriptions ───────────────────────────────────
        self.create_subscription(
            String, '/explorer/scan_status', self._scan_status_cb, 10)

        # ── Service clients ─────────────────────────────────
        # Explorer
        self.panoramic_scan_cli = self.create_client(
            Trigger, '/explorer/panoramic_scan', callback_group=self.cb)

        # Detection pipeline
        self.detect_clear_cli = self.create_client(
            Trigger, '/detection/clear', callback_group=self.cb)
        self.detect_results_cli = self.create_client(
            GetDetectedClusters, '/detection/get_results', callback_group=self.cb)

        # Direct YOLO + depth (for boll-level detection at cluster view)
        self.yolo_detect_cli = self.create_client(
            YoloDetect, '/yolo/detect', callback_group=self.cb)
        self.pixel_to_3d_cli = self.create_client(
            PixelTo3D, '/depth_processor/pixel_to_3d', callback_group=self.cb)

        # Arm commander
        self.go_to_named_cli = self.create_client(
            SetBool, '/go_to_named', callback_group=self.cb)
        self.go_to_pose_cli = self.create_client(
            SetBool, '/go_to_pose', callback_group=self.cb)
        self.arm_set_params_cli = self.create_client(
            SetParameters, '/arm_commander/set_parameters', callback_group=self.cb)

        # Harvest executor
        self.pick_boll_cli = self.create_client(
            HarvestBoll, '/harvest/pick_boll', callback_group=self.cb)

        # Gripper
        self.gripper_open_cli = self.create_client(
            Trigger, '/gripper/open', callback_group=self.cb)

        # ── Services provided ───────────────────────────────
        self.create_service(
            Trigger, '/orchestrator/start_harvest', self._start_harvest_cb,
            callback_group=self.cb)
        self.create_service(
            Trigger, '/orchestrator/stop', self._stop_cb,
            callback_group=self.cb)

        self._set_state(State.IDLE)
        self.get_logger().info('=' * 60)
        self.get_logger().info('HARVEST ORCHESTRATOR ready')
        self.get_logger().info(f'  pre_grasp_offset: {self.pre_grasp_offset}m')
        self.get_logger().info(f'  use_vision_for_scan: {self.use_vision_for_scan}')
        self.get_logger().info(f'  use_vision_for_bolls: {self.use_vision_for_bolls}')
        self.get_logger().info(f'  scan_timeout: {self.scan_timeout}s')
        self.get_logger().info('  Call /orchestrator/start_harvest to begin')
        self.get_logger().info('=' * 60)

    # ─── Config ─────────────────────────────────────────────────

    def _load_config(self) -> dict:
        config_file = self.get_parameter('config_file').value
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            self.get_logger().info(f'Config loaded: {config_file}')
            return config
        self.get_logger().warn('No config file specified, using defaults')
        return {}

    # ─── State management ───────────────────────────────────────

    def _set_state(self, new_state: State):
        old = self.state
        self.state = new_state
        self.status_pub.publish(String(data=new_state.value))
        if old != new_state:
            self.get_logger().info(f'[STATE] {old.value} -> {new_state.value}')

    def _progress(self, msg: str):
        self.progress_pub.publish(String(data=msg))
        self.get_logger().info(f'[PROGRESS] {msg}')

    # ─── Scan status subscription ───────────────────────────────

    def _scan_status_cb(self, msg: String):
        self.get_logger().debug(f'[SCAN STATUS] {msg.data}')
        if msg.data == 'COMPLETE':
            self._scan_complete = True
            self.get_logger().info('[SCAN STATUS] Panoramic scan COMPLETE')

    # ─── Service callbacks ──────────────────────────────────────

    def _start_harvest_cb(self, request, response):
        if self.state != State.IDLE:
            response.success = False
            response.message = f'Cannot start: currently {self.state.value}'
            return response

        self._stop_requested = False
        self.get_logger().info('=' * 60)
        self.get_logger().info('HARVEST CYCLE STARTING')
        self.get_logger().info('=' * 60)

        try:
            self._run_harvest_cycle()
            response.success = True
            response.message = 'Harvest cycle completed'
        except Exception as e:
            self.get_logger().error(f'Harvest cycle FAILED: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self._set_state(State.ERROR)
            response.success = False
            response.message = str(e)

        return response

    def _stop_cb(self, request, response):
        self.get_logger().warn('[STOP] Stop requested')
        self._stop_requested = True
        self._set_state(State.IDLE)
        response.success = True
        response.message = 'Stop requested'
        return response

    def _check_stop(self):
        if self._stop_requested:
            raise RuntimeError('Stop requested by user')

    def _wait_future(self, future, timeout_sec=30.0):
        """Poll-wait for a service future (safe with MultiThreadedExecutor).

        Unlike rclpy.spin_until_future_complete, this does not steal the node
        from the executor — it just sleeps while the executor's other threads
        process the response callback.
        """
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout_sec:
                self.get_logger().warn(
                    f'Service call timed out after {timeout_sec}s')
                return None
            time.sleep(0.05)
        return future.result()

    # ─── Main harvest cycle ─────────────────────────────────────

    def _run_harvest_cycle(self):
        t_start = time.time()

        # ── Phase 1: SCANNING ──────────────────────────────
        self._set_state(State.SCANNING)
        t_phase = time.time()
        cluster_positions = self._phase_scanning()
        self._progress(f'SCAN phase took {time.time()-t_phase:.0f}s wall')
        self._check_stop()

        if not cluster_positions:
            self._progress('No clusters found, returning home')
            self._go_home()
            self._set_state(State.IDLE)
            return

        # ── Build cluster plans ────────────────────────────
        self.cluster_plans = []
        for cid, pos in cluster_positions.items():
            plan = ClusterPlan(cluster_id=cid, center=list(pos))
            self.cluster_plans.append(plan)

        self._progress(
            f'Found {len(self.cluster_plans)} clusters: '
            f'{[p.cluster_id for p in self.cluster_plans]}')
        for p in self.cluster_plans:
            self.get_logger().info(
                f'  {p.cluster_id}: center=({p.center[0]:.3f}, '
                f'{p.center[1]:.3f}, {p.center[2]:.3f})')

        # ── Phase 2: Per-cluster harvest ───────────────────
        for idx, plan in enumerate(self.cluster_plans):
            self._check_stop()
            self.current_cluster_idx = idx
            t_cluster = time.time()
            self._progress(
                f'=== Cluster {idx+1}/{len(self.cluster_plans)}: '
                f'{plan.cluster_id} at ({plan.center[0]:.3f}, '
                f'{plan.center[1]:.3f}, {plan.center[2]:.3f}) ===')

            # 2a. APPROACHING
            self._set_state(State.APPROACHING)
            t_phase = time.time()
            self._phase_approaching(plan)
            self._progress(
                f'APPROACH phase took {time.time()-t_phase:.0f}s wall, '
                f'{len(plan.bolls)} boll(s) found')
            self._check_stop()

            # 2b. HARVESTING
            self._set_state(State.HARVESTING)
            t_phase = time.time()
            self._phase_harvesting(plan)
            picked_in_cluster = sum(1 for b in plan.bolls if b.picked)
            self._progress(
                f'HARVEST phase took {time.time()-t_phase:.0f}s wall, '
                f'{picked_in_cluster}/{len(plan.bolls)} picked')

            plan.completed = True
            self._progress(
                f'=== {plan.cluster_id} DONE in '
                f'{time.time()-t_cluster:.0f}s ===')

        # ── Phase 3: RETURNING ─────────────────────────────
        self._set_state(State.RETURNING)
        self._progress('All clusters done, returning HOME')
        t_phase = time.time()
        self._go_home()
        self._progress(f'HOME reached in {time.time()-t_phase:.0f}s')

        # ── Summary ────────────────────────────────────────
        elapsed = time.time() - t_start
        picked = sum(1 for p in self.cluster_plans for b in p.bolls if b.picked)
        total = sum(len(p.bolls) for p in self.cluster_plans)
        self.get_logger().info('=' * 60)
        self._progress(
            f'HARVEST COMPLETE: {picked}/{total} bolls, '
            f'{elapsed:.0f}s wall time')
        for p in self.cluster_plans:
            p_picked = sum(1 for b in p.bolls if b.picked)
            self.get_logger().info(
                f'  {p.cluster_id}: {p_picked}/{len(p.bolls)} bolls picked')
        self.get_logger().info('=' * 60)
        self._set_state(State.IDLE)

    # ─── Phase: SCANNING ────────────────────────────────────────

    def _phase_scanning(self) -> dict:
        """
        Scan field and return cluster positions.

        With vision: explorer panoramic scan + detection pipeline
        Without vision: read from config (fallback)
        """
        if not self.use_vision_for_scan:
            self._progress('Scan: using config positions (vision disabled)')
            return self._get_clusters_from_config()

        self._progress('Scan: starting panoramic scan with detection...')

        # Step 1: Clear previous detections (arm already at cluster-facing view from startup)
        self._progress('Scan: clearing previous detections')
        self._call_trigger(self.detect_clear_cli, '/detection/clear')

        # Step 3: Start panoramic scan
        self._scan_complete = False
        self._progress('Scan: starting panoramic scan (3 positions)')
        scan_ok = self._call_trigger(
            self.panoramic_scan_cli, '/explorer/panoramic_scan')
        if not scan_ok:
            self.get_logger().warn(
                'Scan: panoramic_scan service failed, falling back to config')
            return self._get_clusters_from_config()

        # Step 4: Wait for scan completion
        self._progress('Scan: waiting for scan to complete...')
        t_start = time.time()
        while not self._scan_complete:
            if time.time() - t_start > self.scan_timeout:
                self.get_logger().error(
                    f'Scan: timeout after {self.scan_timeout}s, '
                    f'falling back to config')
                return self._get_clusters_from_config()
            self._check_stop()
            time.sleep(1.0)

        elapsed = time.time() - t_start
        self._progress(f'Scan: completed in {elapsed:.0f}s wall time')

        # Step 5: Get detection results (log them for demo)
        self._progress('Scan: retrieving detection results...')
        results = self._call_get_results()

        if results:
            for cluster in results:
                pos = [cluster.position.x, cluster.position.y, cluster.position.z]
                self._progress(
                    f'Scan: detected {cluster.cluster_id} at '
                    f'({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}), '
                    f'confidence={cluster.confidence:.2f}, '
                    f'detections={cluster.num_detections}')
            self._progress(f'Scan: {len(results)} clusters detected by vision')
        else:
            self._progress('Scan: no clusters detected by vision')

        # For demo reliability, always use config positions
        self._progress('Scan: using config positions for demo')
        return self._get_clusters_from_config()

    def _get_clusters_from_config(self) -> dict:
        """Fallback: get cluster positions from config file."""
        clusters = self.config.get('clusters', {})
        result = {}
        for cid, cdata in clusters.items():
            pos = cdata.get('position', [0, 0, 0])
            result[cid] = pos
            self.get_logger().info(f'  Config cluster {cid}: {pos}')
        return result

    # ─── Phase: APPROACHING ─────────────────────────────────────

    def _phase_approaching(self, plan: ClusterPlan):
        """
        Approach cluster and detect individual bolls.

        1. Go to pre-grasp cluster view (arm_commander handles offset)
        2. Wait for camera to settle
        3. Run YOLO detect + depth for individual boll 3D positions
        4. Fallback to cluster center if detection fails
        """
        cx, cy, cz = plan.center

        # Step 1: Compute pre-grasp view position
        pre_x, pre_y, pre_z = self._compute_pre_grasp(cx, cy, cz)
        plan.pre_grasp_view = [pre_x, pre_y, pre_z]

        self._progress(
            f'Approach {plan.cluster_id}: going to pre-grasp view '
            f'({pre_x:.3f}, {pre_y:.3f}, {pre_z:.3f})')

        # Go to pre-grasp view with approach orientation toward cluster
        success = self._go_to_xyz(pre_x, pre_y, pre_z, approach_orientation=True)
        if not success:
            self.get_logger().error(
                f'Approach {plan.cluster_id}: failed to reach pre-grasp view')
            # Still try with config position as fallback boll
            plan.bolls = [BollTarget(
                position=plan.center.copy(), cluster_id=plan.cluster_id)]
            return

        # Step 2: Wait for camera to settle
        settle = self.camera_settle_time
        self._progress(
            f'Approach {plan.cluster_id}: at pre-grasp, '
            f'waiting {settle}s for camera...')
        time.sleep(settle)

        # Step 3: Detect individual bolls
        if self.use_vision_for_bolls:
            self._progress(
                f'Approach {plan.cluster_id}: running boll detection...')
            bolls = self._detect_bolls()

            if bolls:
                plan.bolls = bolls
                self._progress(
                    f'Approach {plan.cluster_id}: detected {len(bolls)} boll(s)')
                for i, b in enumerate(bolls):
                    self.get_logger().info(
                        f'  Boll {i+1}: ({b.position[0]:.3f}, '
                        f'{b.position[1]:.3f}, {b.position[2]:.3f}), '
                        f'conf={b.confidence:.2f}')
            else:
                self.get_logger().warn(
                    f'Approach {plan.cluster_id}: '
                    f'no bolls detected, using cluster center')
                plan.bolls = [BollTarget(
                    position=plan.center.copy(),
                    cluster_id=plan.cluster_id)]
        else:
            self._progress(
                f'Approach {plan.cluster_id}: '
                f'vision disabled, using cluster center')
            plan.bolls = [BollTarget(
                position=plan.center.copy(),
                cluster_id=plan.cluster_id)]

        self._progress(
            f'Approach {plan.cluster_id}: '
            f'{len(plan.bolls)} boll(s) to pick')

    def _detect_bolls(self) -> List[BollTarget]:
        """
        Detect individual bolls at current camera view.
        Calls /yolo/detect (raw) then /depth_processor/pixel_to_3d for each.
        """
        # Call YOLO raw detection
        if not self.yolo_detect_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('_detect_bolls: /yolo/detect not available')
            return []

        yolo_req = YoloDetect.Request()
        future = self.yolo_detect_cli.call_async(yolo_req)
        self._wait_future(future,10.0)

        if future.result() is None:
            self.get_logger().error('_detect_bolls: YOLO call returned None')
            return []

        yolo_result = future.result()
        if not yolo_result.success:
            self.get_logger().warn(
                f'_detect_bolls: YOLO failed: {yolo_result.message}')
            return []

        detections = yolo_result.detections
        self.get_logger().info(
            f'_detect_bolls: YOLO returned {len(detections)} detections')

        if not detections:
            return []

        # For each detection, get 3D position via depth
        bolls = []
        for i, bbox in enumerate(detections):
            # Filter: only cotton_boll class
            if 'cotton' not in bbox.label.lower() and 'boll' not in bbox.label.lower():
                self.get_logger().info(
                    f'  Detection {i}: skipping non-boll label={bbox.label}')
                continue

            cx = (bbox.u_min + bbox.u_max) // 2
            cy = (bbox.v_min + bbox.v_max) // 2

            self.get_logger().info(
                f'  Detection {i}: {bbox.label} conf={bbox.confidence:.2f}, '
                f'pixel=({cx}, {cy}), area={bbox.area}')

            # Call depth processor
            pos_3d = self._call_pixel_to_3d(cx, cy)
            if pos_3d is None:
                self.get_logger().warn(
                    f'  Detection {i}: depth lookup failed, skipping')
                continue

            self.get_logger().info(
                f'  Detection {i}: 3D position = '
                f'({pos_3d[0]:.3f}, {pos_3d[1]:.3f}, {pos_3d[2]:.3f})')

            bolls.append(BollTarget(
                position=pos_3d,
                cluster_id='detected',
                confidence=bbox.confidence))

        self.get_logger().info(
            f'_detect_bolls: {len(bolls)} bolls with valid 3D positions')
        return bolls

    def _call_pixel_to_3d(self, u: int, v: int) -> Optional[List[float]]:
        """Call /depth_processor/pixel_to_3d, return [x,y,z] or None."""
        if not self.pixel_to_3d_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('pixel_to_3d service not available')
            return None

        req = PixelTo3D.Request()
        req.u = u
        req.v = v

        future = self.pixel_to_3d_cli.call_async(req)
        self._wait_future(future,10.0)

        if future.result() is None:
            self.get_logger().warn(f'pixel_to_3d({u},{v}): call returned None')
            return None

        result = future.result()
        if not result.success:
            self.get_logger().warn(
                f'pixel_to_3d({u},{v}): {result.message}')
            return None

        return [result.position.x, result.position.y, result.position.z]

    # ─── Phase: HARVESTING ──────────────────────────────────────

    def _phase_harvesting(self, plan: ClusterPlan):
        """Pick all bolls in a cluster via harvest_executor."""
        if not plan.bolls:
            self._progress(f'Harvest {plan.cluster_id}: no bolls to pick')
            return

        self._progress(
            f'Harvest {plan.cluster_id}: picking {len(plan.bolls)} boll(s)')

        for i, boll in enumerate(plan.bolls):
            self._check_stop()

            if boll.picked:
                continue

            self._progress(
                f'Harvest {plan.cluster_id}: boll {i+1}/{len(plan.bolls)} '
                f'at ({boll.position[0]:.3f}, {boll.position[1]:.3f}, '
                f'{boll.position[2]:.3f})')

            success = self._pick_single_boll(boll, plan.pre_grasp_view)
            boll.picked = success

            if success:
                self._progress(
                    f'Harvest {plan.cluster_id}: '
                    f'boll {i+1} PICKED successfully')
            else:
                self._progress(
                    f'Harvest {plan.cluster_id}: '
                    f'boll {i+1} FAILED, continuing to next')

        picked = sum(1 for b in plan.bolls if b.picked)
        self._progress(
            f'Harvest {plan.cluster_id}: '
            f'{picked}/{len(plan.bolls)} bolls picked')

    # ─── Arm helpers ────────────────────────────────────────────

    def _go_to_xyz(self, x, y, z, approach_orientation=False) -> bool:
        """Set arm_commander params and call /go_to_pose."""
        self.get_logger().info(
            f'[ARM] go_to_xyz({x:.3f}, {y:.3f}, {z:.3f},'
            f' approach={approach_orientation})')

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
                          bool_value=False)),
        ]
        if not self._set_arm_params(params):
            return False

        if not self.go_to_pose_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('[ARM] /go_to_pose not available')
            return False
        future = self.go_to_pose_cli.call_async(SetBool.Request(data=True))
        self._wait_future(future,180.0)
        if future.result() is not None:
            ok = future.result().success
            self.get_logger().info(
                f'[ARM] go_to_xyz result: {"OK" if ok else "FAIL"} '
                f'- {future.result().message}')
            return ok
        self.get_logger().error('[ARM] go_to_xyz timeout')
        return False

    def _go_to_named(self, name: str) -> bool:
        """Set target_name and call /go_to_named."""
        self.get_logger().info(f'[ARM] go_to_named({name})')

        params = [
            Parameter(name='target_name',
                      value=ParameterValue(
                          type=ParameterType.PARAMETER_STRING,
                          string_value=name)),
        ]
        if not self._set_arm_params(params):
            return False

        if not self.go_to_named_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('[ARM] /go_to_named not available')
            return False
        future = self.go_to_named_cli.call_async(SetBool.Request(data=True))
        self._wait_future(future,120.0)
        if future.result() is not None:
            ok = future.result().success
            self.get_logger().info(
                f'[ARM] go_to_named({name}) result: {"OK" if ok else "FAIL"} '
                f'- {future.result().message}')
            return ok
        self.get_logger().error(f'[ARM] go_to_named({name}) timeout')
        return False

    def _go_home(self) -> bool:
        """Go HOME via /go_to_named."""
        self._progress('Going HOME...')
        return self._go_to_named('home')

    def _set_arm_params(self, params) -> bool:
        """Set parameters on arm_commander."""
        if not self.arm_set_params_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('[ARM] set_parameters not available')
            return False
        req = SetParameters.Request(parameters=params)
        future = self.arm_set_params_cli.call_async(req)
        self._wait_future(future,5.0)
        if future.result() is None:
            self.get_logger().error('[ARM] set_parameters returned None')
            return False
        if not all(r.successful for r in future.result().results):
            self.get_logger().error('[ARM] set_parameters failed')
            return False
        self.get_logger().debug('[ARM] set_parameters OK')
        return True

    # ─── Harvest executor helper ────────────────────────────────

    def _pick_single_boll(self, boll: BollTarget,
                          pre_grasp_view: List[float]) -> bool:
        """Call /harvest/pick_boll service."""
        self.get_logger().info(
            f'[PICK] Calling /harvest/pick_boll: '
            f'boll=({boll.position[0]:.3f}, {boll.position[1]:.3f}, '
            f'{boll.position[2]:.3f}), '
            f'pre_grasp=({pre_grasp_view[0]:.3f}, '
            f'{pre_grasp_view[1]:.3f}, {pre_grasp_view[2]:.3f})')

        if not self.pick_boll_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('[PICK] /harvest/pick_boll not available')
            return False

        req = HarvestBoll.Request()
        req.boll_position = Point(
            x=boll.position[0], y=boll.position[1], z=boll.position[2])
        req.pre_grasp_position = Point(
            x=pre_grasp_view[0], y=pre_grasp_view[1], z=pre_grasp_view[2])

        future = self.pick_boll_cli.call_async(req)
        self._wait_future(future,600.0)

        if future.result() is not None:
            ok = future.result().success
            msg = future.result().message
            self.get_logger().info(
                f'[PICK] Result: {"OK" if ok else "FAIL"} - {msg}')
            return ok
        self.get_logger().error('[PICK] /harvest/pick_boll timeout')
        return False

    # ─── Service call helpers ───────────────────────────────────

    def _call_trigger(self, client, name: str) -> bool:
        """Generic trigger service call."""
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'{name} not available')
            return False
        future = client.call_async(Trigger.Request())
        self._wait_future(future,30.0)
        if future.result() is not None:
            ok = future.result().success
            msg = future.result().message
            self.get_logger().info(f'{name}: {"OK" if ok else "FAIL"} - {msg}')
            return ok
        self.get_logger().error(f'{name}: timeout')
        return False

    def _call_get_results(self):
        """Call /detection/get_results and return cluster list."""
        if not self.detect_results_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/detection/get_results not available')
            return []
        req = GetDetectedClusters.Request()
        future = self.detect_results_cli.call_async(req)
        self._wait_future(future,10.0)
        if future.result() is None:
            self.get_logger().error('/detection/get_results returned None')
            return []
        result = future.result()
        if not result.success:
            self.get_logger().warn(
                f'/detection/get_results failed: {result.message}')
            return []
        self.get_logger().info(
            f'/detection/get_results: {len(result.clusters)} clusters')
        return list(result.clusters)

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


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
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
