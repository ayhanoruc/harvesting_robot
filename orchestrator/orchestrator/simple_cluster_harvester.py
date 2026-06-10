#!/usr/bin/env python3
"""
Simple Cluster Harvester (F1.7 demo)

Standalone, simplified pick cycle for a single cluster (tree). NO pre-grasp,
NO approach standoff, NO real gripper — just direct go-to-boll, mock close,
teleport to reservoir.

Different from harvest_executor (which is the full 8-step pipeline used by the
state-machine orchestrator); this module is a quick demo for F1.7.

Inputs
------
  - orchard_bolls.yaml — ground-truth boll positions (from generate_bolls.py)

Behavior
--------
  Service /simple_harvest/start (std_srvs/Trigger):
    1. Move arm to HOME (via /go_to_named).
    2. For each boll belonging to `tree_id` parameter:
         a. arm_commander.go_to_xyz(boll.x, boll.y, boll.z, approach_orientation=True)
         b. log "[MOCK GRIP CLOSE]"
         c. teleport boll model to TCP via Gazebo /world/.../set_pose
         d. arm_commander.go_to_xyz(reservoir x, y, z+hover, approach_orientation=True)
         e. log "[MOCK GRIP OPEN]"
         f. teleport boll model to reservoir drop (with grid scatter)
    3. Move arm back to HOME.

Parameters
----------
  tree_id              : 'tree_000'   — which cluster's bolls to harvest
  boll_inventory_yaml  : ''           — path; defaults to robot_arm/share orchard_bolls.yaml
  gz_world_name        : 'orchard'
  tcp_frame            : 'tcp'
  world_frame          : 'world'
  reservoir_tf_frame   : 'reservoir_link'
  reservoir_drop_clearance_m : 0.12
  reservoir_hover_m    : 0.15        — Z above reservoir top during release pose
  mock_close_delay_s   : 0.5
  mock_open_delay_s    : 0.3

Usage
-----
  Terminal A: ros2 launch robot_arm husky_test.launch.py
  Terminal B: ros2 launch robot_arm_moveit_config moveit.launch.py
  Terminal C: ros2 run orchestrator simple_cluster_harvester
              ros2 service call /simple_harvest/start std_srvs/srv/Trigger '{}'

  Override target tree:
    ros2 param set /simple_cluster_harvester tree_id tree_005
"""

from __future__ import annotations

import math
import os
import shutil
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import List, Optional, Tuple

import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import yaml
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from tf2_ros import Buffer, TransformListener

from ament_index_python.packages import get_package_share_directory

# Optional fast-path teleport: a persistent rclpy client on the Gazebo
# set_pose service bridged via ros_gz_bridge. If ros_gz_interfaces isn't
# importable, _HAVE_SETPOSE_SRV stays False and _gz_set_pose silently uses
# the original `ign service` subprocess fallback — no behaviour change.
try:
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.msg import Entity
    _HAVE_SETPOSE_SRV = True
except Exception:
    _HAVE_SETPOSE_SRV = False


class SimpleClusterHarvester(Node):

    def __init__(self):
        super().__init__('simple_cluster_harvester')
        self.cb = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────
        # Defaults point at cotton_demo world (template-instanced compact).
        # Override via launch param or `ros2 param set` to use legacy
        # orchard_bolls world (tree_000 / 'orchard' / orchard_bolls.yaml).
        self.declare_parameter('tree_id', 'cluster_A_01')
        # Optional runtime override: if non-empty, harvest exactly these
        # boll IDs (looked up from the YAML inventory) instead of *all*
        # bolls for `tree_id`. Used by cluster_harvester to feed in the
        # subset returned by /cluster_scan/run (detected, not ground-truth).
        self.declare_parameter('boll_ids_runtime', [''])
        self.declare_parameter('boll_inventory_yaml', '')
        self.declare_parameter('gz_world_name', 'cotton_demo')
        self.declare_parameter('tcp_frame', 'tcp')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('reservoir_tf_frame', 'reservoir_link')
        self.declare_parameter('reservoir_drop_clearance_m', 0.12)
        self.declare_parameter('reservoir_hover_m', 0.15)
        self.declare_parameter('mock_close_delay_s', 0.5)
        self.declare_parameter('mock_open_delay_s', 0.3)

        # ── State ───────────────────────────────────────────────
        self._busy = False
        self._reservoir_pos_default = [0.0, 0.6, 0.3]
        self._drop_slot = 0  # incremented per successful drop for grid scatter

        # Continuous-carry thread state (boll-glued-to-TCP during pick)
        self._carry_lock = threading.Lock()
        self._carry_model_name: Optional[str] = None
        self._carry_thread: Optional[threading.Thread] = None
        self._carry_active = False
        # 50 Hz so the carried boll visually tracks the TCP without lag.
        # Achievable because the bridged set_pose client is fire-and-forget
        # (~1 ms/call); with the subprocess fallback the loop self-limits to
        # whatever the CLI can sustain, same as before.
        self._carry_rate_hz = 50.0

        # Reservoir-carry thread state (already-dropped bolls stay glued to
        # reservoir_link as Husky drives between clusters). We store each
        # boll's *local* offset in reservoir frame, then on each tick re-apply
        # world_pose = reservoir_world * local_offset.
        self._dropped_lock = threading.Lock()
        self._dropped_items: List[Tuple[str, Tuple[float, float, float]]] = []
        self._dropped_carry_thread: Optional[threading.Thread] = None
        self._dropped_carry_active = False
        self._dropped_carry_rate_hz = 2.0
        self._dropped_motion_threshold_m = 0.05  # only snap if reservoir moved
        # Parallel subprocess pool — set_pose via `ign service` is ~80ms each;
        # batching with workers keeps per-tick latency manageable for ~20 bolls
        self._gz_pool = ThreadPoolExecutor(max_workers=4)

        # ── Boll inventory ──────────────────────────────────────
        self._boll_items: List[dict] = self._load_boll_inventory()
        if not self._boll_items:
            self.get_logger().warn(
                'No boll inventory loaded. Service will fail until generate_bolls.py runs.')

        # ── TF ──────────────────────────────────────────────────
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # ── Service clients (arm_commander) ─────────────────────
        self.go_to_pose_cli = self.create_client(
            SetBool, '/go_to_pose', callback_group=self.cb)
        self.go_to_named_cli = self.create_client(
            SetBool, '/go_to_named', callback_group=self.cb)
        self.go_to_reservoir_cli = self.create_client(
            SetBool, '/go_to_reservoir', callback_group=self.cb)
        self.arm_set_params_cli = self.create_client(
            SetParameters, '/arm_commander/set_parameters', callback_group=self.cb)

        # ── Gripper service clients ─────────────────────────────
        # The pick stays MOCK (teleport-based — boll moves into the bin
        # by set_pose, not by real grasp physics) but we ALSO command the
        # gripper to open/close visually so the user can see the fingers
        # animate in Gazebo. These calls don't affect the boll's motion.
        self.gripper_open_cli = self.create_client(
            Trigger, '/gripper/open', callback_group=self.cb)
        self.gripper_close_cli = self.create_client(
            Trigger, '/gripper/close', callback_group=self.cb)

        # ── Fast teleport client (set_pose bridged via ros_gz_bridge) ──
        # Persistent client → ~1 ms per call vs ~80 ms for the `ign service`
        # CLI subprocess. Only used by _gz_set_pose when the bridge is up;
        # otherwise the subprocess fallback runs unchanged.
        self._setpose_cli = None
        if _HAVE_SETPOSE_SRV:
            world = self.get_parameter('gz_world_name').value
            self._setpose_cli = self.create_client(
                SetEntityPose, f'/world/{world}/set_pose',
                callback_group=self.cb)

        # ── Status publisher ────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/simple_harvest/status', 10)

        # ── Service ─────────────────────────────────────────────
        self.create_service(
            Trigger, '/simple_harvest/start', self._on_start,
            callback_group=self.cb)

        self.get_logger().info('=' * 60)
        self.get_logger().info('SIMPLE CLUSTER HARVESTER ready')
        self.get_logger().info(f'  bolls loaded: {len(self._boll_items)}')
        self.get_logger().info(f'  default tree_id: {self.get_parameter("tree_id").value}')
        self.get_logger().info(f'  service: /simple_harvest/start')
        self.get_logger().info('=' * 60)

    # ─── Inventory ──────────────────────────────────────────────

    def _load_boll_inventory(self) -> List[dict]:
        path = self.get_parameter('boll_inventory_yaml').value
        if not path:
            try:
                share = get_package_share_directory('robot_arm')
                # Default: cotton_demo (template-instanced compact world).
                # Falls back to orchard_bolls.yaml for legacy worlds.
                cand = os.path.join(share, 'config', 'cotton_demo_bolls.yaml')
                path = cand if os.path.isfile(cand) else \
                       os.path.join(share, 'config', 'orchard_bolls.yaml')
            except Exception:
                path = ''
        if not path or not os.path.isfile(path):
            self.get_logger().warn(f'boll inventory yaml not found: {path}')
            return []
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            items = data.get('items', []) or []
            self.get_logger().info(f'Loaded {len(items)} bolls from {path}')
            return items
        except Exception as e:
            self.get_logger().error(f'Failed to read {path}: {e}')
            return []

    def _bolls_for_tree(self, tree_id: str) -> List[dict]:
        return [b for b in self._boll_items if b.get('tree_id') == tree_id]

    # ─── TF helper ──────────────────────────────────────────────

    def _world_pose_of_frame(self, frame_id: str):
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter('world_frame').value, frame_id,
                rclpy.time.Time(), timeout=Duration(seconds=2.0))
            tr = t.transform.translation
            q = t.transform.rotation
            return (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)
        except Exception as ex:
            self.get_logger().warn(f'TF world<-{frame_id}: {ex}')
            return None

    # ─── Gazebo set_pose teleport ───────────────────────────────

    def _gz_set_pose(self, model_name: str, x: float, y: float, z: float,
                    qx: float = 0.0, qy: float = 0.0,
                    qz: float = 0.0, qw: float = 1.0) -> bool:
        # Fast path: persistent client on the bridged set_pose service.
        # Fire-and-forget — the carry loop snaps many times a second and
        # doesn't need the boolean ack. Falls through to the subprocess
        # below whenever the bridge isn't up (service_is_ready False).
        cli = self._setpose_cli
        if cli is not None and cli.service_is_ready():
            req = SetEntityPose.Request()
            req.entity.name = model_name
            req.entity.type = Entity.MODEL
            req.pose.position.x = float(x)
            req.pose.position.y = float(y)
            req.pose.position.z = float(z)
            req.pose.orientation.x = float(qx)
            req.pose.orientation.y = float(qy)
            req.pose.orientation.z = float(qz)
            req.pose.orientation.w = float(qw)
            cli.call_async(req)
            return True

        world = self.get_parameter('gz_world_name').value
        srv = f'/world/{world}/set_pose'
        req_txt = (
            f'name: "{model_name}"\n'
            f'position {{ x: {x} y: {y} z: {z} }}\n'
            f'orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }}\n'
        )
        cli_args = ['service',
                    '-s', srv,
                    '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '2500',
                    '--req', req_txt]
        for exe in ('ign', 'gz'):
            exe_path = shutil.which(exe)
            if not exe_path:
                continue
            try:
                ret = subprocess.run(
                    [exe_path] + cli_args,
                    capture_output=True, text=True,
                    timeout=6.0, check=False)
                out = (ret.stdout or '') + (ret.stderr or '')
                if ret.returncode == 0 and 'true' in out.lower():
                    self.get_logger().info(
                        f'[GZ] set_pose {model_name} → '
                        f'({x:.3f}, {y:.3f}, {z:.3f}) via {exe}')
                    return True
                self.get_logger().warn(
                    f'[GZ] {exe} set_pose rc={ret.returncode}: {out[:200]}')
            except Exception as e:
                self.get_logger().warn(f'[GZ] {exe} subprocess: {e}')
        self.get_logger().error('[GZ] No ign/gz on PATH; teleport disabled.')
        return False

    def _teleport_to_tcp(self, model_name: str) -> bool:
        p = self._world_pose_of_frame(self.get_parameter('tcp_frame').value)
        if not p:
            return False
        return self._gz_set_pose(
            model_name, p[0], p[1], p[2],
            qx=p[3], qy=p[4], qz=p[5], qw=p[6])

    # ─── Continuous "carry" thread: keeps boll glued to TCP during arm motion ──

    def _carry_loop(self):
        """Background loop: while active, snap carry model to TCP at carry_rate_hz."""
        period = 1.0 / max(1.0, self._carry_rate_hz)
        tcp_frame = self.get_parameter('tcp_frame').value
        while True:
            with self._carry_lock:
                active = self._carry_active
                model = self._carry_model_name
            if not active or not model:
                break
            p = self._world_pose_of_frame(tcp_frame)
            if p:
                # Don't log every snap — silent unless _gz_set_pose failure spam
                self._gz_set_pose(
                    model, p[0], p[1], p[2],
                    qx=p[3], qy=p[4], qz=p[5], qw=p[6])
            time.sleep(period)

    def _carry_start(self, model_name: str):
        with self._carry_lock:
            self._carry_model_name = model_name
            self._carry_active = True
        self._carry_thread = threading.Thread(target=self._carry_loop, daemon=True)
        self._carry_thread.start()
        self.get_logger().info(f'[CARRY] continuous TCP-snap started for {model_name}')

    def _carry_stop(self):
        with self._carry_lock:
            self._carry_active = False
            model = self._carry_model_name
            self._carry_model_name = None
        if self._carry_thread is not None:
            self._carry_thread.join(timeout=1.0)
        self._carry_thread = None
        if model:
            self.get_logger().info(f'[CARRY] continuous TCP-snap stopped (was {model})')

    @staticmethod
    def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def _reservoir_local_to_world(
            self, reservoir_pose, lx: float, ly: float, lz: float):
        """Apply reservoir's world pose (yaw only — bin stays upright) to a
        point given in reservoir-local coordinates."""
        rx, ry, rz, qx, qy, qz, qw = reservoir_pose
        yaw = self._yaw_from_quat(qx, qy, qz, qw)
        cy, sy = math.cos(yaw), math.sin(yaw)
        wx = rx + cy * lx - sy * ly
        wy = ry + sy * lx + cy * ly
        wz = rz + lz
        return (wx, wy, wz)

    def _teleport_to_reservoir(self, model_name: str) -> bool:
        """Drop boll into reservoir bin and register it for Husky-follow."""
        rp = self._world_pose_of_frame(self.get_parameter('reservoir_tf_frame').value)
        clearance = float(self.get_parameter('reservoir_drop_clearance_m').value)
        if not rp:
            # Fallback: no reservoir TF (shouldn't happen on Husky URDF) —
            # use static default and skip Husky-follow registration.
            d = self._reservoir_pos_default
            x, y, z = d[0], d[1], d[2] + clearance
            return self._gz_set_pose(model_name, x, y, z)

        # Local offset in reservoir frame: 5×N grid scatter, hovering above
        # bin floor by `clearance`. These coords stay constant — only the
        # world pose changes as Husky moves.
        slot = self._drop_slot
        col = slot % 5
        row = slot // 5
        lx = -0.08 + 0.04 * col
        ly = -0.08 + 0.04 * row
        lz = clearance

        wx, wy, wz = self._reservoir_local_to_world(rp, lx, ly, lz)
        ok = self._gz_set_pose(model_name, wx, wy, wz)
        if ok:
            self._drop_slot += 1
            with self._dropped_lock:
                self._dropped_items.append((model_name, (lx, ly, lz)))
            self._start_dropped_carry()
        return ok

    # ─── Reservoir-carry: keep dropped bolls glued to reservoir as Husky moves ──

    def _dropped_carry_loop(self):
        """Background loop: re-snap each registered boll to its reservoir-local
        position whenever the reservoir has moved more than the threshold.
        Snaps run in parallel via _gz_pool to keep per-tick latency bounded."""
        period = 1.0 / max(0.5, self._dropped_carry_rate_hz)
        last_pose = None
        rsv_frame = self.get_parameter('reservoir_tf_frame').value
        while True:
            with self._dropped_lock:
                active = self._dropped_carry_active
                items = list(self._dropped_items)
            if not active:
                break
            if items:
                rp = self._world_pose_of_frame(rsv_frame)
                if rp is not None:
                    moved_enough = (
                        last_pose is None
                        or math.hypot(rp[0] - last_pose[0],
                                      rp[1] - last_pose[1])
                        > self._dropped_motion_threshold_m)
                    if moved_enough:
                        futures = []
                        for model, local in items:
                            wx, wy, wz = self._reservoir_local_to_world(
                                rp, *local)
                            futures.append(self._gz_pool.submit(
                                self._gz_set_pose, model, wx, wy, wz))
                        # Drain batch — don't accumulate backlog if a tick is
                        # slower than the period
                        for f in futures:
                            try:
                                f.result(timeout=2.0)
                            except Exception:
                                pass
                        last_pose = rp
            time.sleep(period)

    def _start_dropped_carry(self):
        with self._dropped_lock:
            if self._dropped_carry_active:
                return
            self._dropped_carry_active = True
        self._dropped_carry_thread = threading.Thread(
            target=self._dropped_carry_loop, daemon=True)
        self._dropped_carry_thread.start()
        self.get_logger().info(
            '[RSV-CARRY] reservoir-frame snap thread started '
            f'({self._dropped_carry_rate_hz:.1f} Hz, '
            f'motion threshold {self._dropped_motion_threshold_m:.2f}m)')

    # ─── Arm helpers (mirror of harvest_executor pattern) ──────

    def _set_arm_params(self, params) -> bool:
        if not self.arm_set_params_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('[ARM] set_parameters service unavailable')
            return False
        req = SetParameters.Request(parameters=params)
        future = self.arm_set_params_cli.call_async(req)
        self._wait_future(future, 5.0)
        if future.result() is None:
            return False
        return all(r.successful for r in future.result().results)

    def _go_to_xyz(self, x: float, y: float, z: float,
                   approach_orientation: bool = True,
                   use_direct: bool = False) -> bool:
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
        if not self._set_arm_params(params):
            return False
        if not self.go_to_pose_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('[ARM] /go_to_pose unavailable')
            return False
        future = self.go_to_pose_cli.call_async(SetBool.Request(data=True))
        self._wait_future(future, 120.0)
        if future.result() is None:
            return False
        return future.result().success

    def _go_home(self) -> bool:
        params = [
            Parameter(name='target_name',
                      value=ParameterValue(
                          type=ParameterType.PARAMETER_STRING, string_value='home')),
        ]
        if not self._set_arm_params(params):
            return False
        if not self.go_to_named_cli.wait_for_service(timeout_sec=5.0):
            return False
        future = self.go_to_named_cli.call_async(SetBool.Request(data=True))
        self._wait_future(future, 120.0)
        return (future.result() is not None and future.result().success)

    def _go_reservoir(self) -> bool:
        """Heuristic 3-stage reach to reservoir hover (HOME → joint1 rotate → IK).

        Implemented in arm_commander.go_to_reservoir_callback. Avoids the
        single-shot 'reach behind' IK problem by first rotating the base so
        the arm physically faces the reservoir.
        """
        if not self.go_to_reservoir_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('[ARM] /go_to_reservoir unavailable')
            return False
        future = self.go_to_reservoir_cli.call_async(SetBool.Request(data=True))
        self._wait_future(future, 180.0)
        return (future.result() is not None and future.result().success)

    def _gripper_call(self, client, label: str) -> bool:
        """Fire-and-block call to /gripper/open or /gripper/close.

        We *don't* care if it fails (the pick is mock-teleport — boll motion
        doesn't depend on gripper physics). The whole point is just to see
        the fingers animate in Gazebo. Logs the outcome and moves on.
        """
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'[GRIP] /{label} service unavailable, skipping')
            return False
        future = client.call_async(Trigger.Request())
        # Gripper motion is fast (~0.4s default); 5s is generous.
        result = self._wait_future(future, 5.0)
        if result is None:
            self.get_logger().warn(f'[GRIP] /{label} timeout')
            return False
        if not result.success:
            self.get_logger().warn(f'[GRIP] /{label} reported failure: {result.message}')
            return False
        return True

    def _wait_future(self, future, timeout_sec: float) -> Optional[object]:
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout_sec:
                return None
            time.sleep(0.05)
        return future.result()

    # ─── Status ─────────────────────────────────────────────────

    def _publish_status(self, msg: str):
        self.status_pub.publish(String(data=msg))
        self.get_logger().info(f'[STATUS] {msg}')

    # ─── Main entry: harvest one cluster ────────────────────────

    def _on_start(self, request, response):
        if self._busy:
            response.success = False
            response.message = 'Already harvesting'
            return response

        # Boll source: runtime ID override > YAML by tree_id.
        runtime_ids = [
            i for i in (self.get_parameter('boll_ids_runtime').value or [])
            if i  # drop empty strings (default '[""]' placeholder)
        ]
        if runtime_ids:
            id_set = set(runtime_ids)
            bolls_unsorted = [b for b in self._boll_items if b.get('id') in id_set]
            # Preserve caller's requested order (cluster_harvester sorts closest-first)
            order = {bid: i for i, bid in enumerate(runtime_ids)}
            bolls = sorted(bolls_unsorted, key=lambda b: order.get(b['id'], 1_000_000))
            tree_id = f'runtime[{len(bolls)}]'
            missing = [bid for bid in runtime_ids if bid not in {b['id'] for b in bolls}]
            if missing:
                self.get_logger().warn(
                    f'runtime IDs missing in inventory: {missing}')
            if not bolls:
                response.success = False
                response.message = f'No matching bolls for runtime IDs: {runtime_ids}'
                self.get_logger().error(response.message)
                return response
        else:
            tree_id = self.get_parameter('tree_id').value
            bolls = self._bolls_for_tree(tree_id)
            if not bolls:
                response.success = False
                response.message = (
                    f'No bolls found for tree_id="{tree_id}". '
                    f'Check orchard_bolls.yaml.')
                self.get_logger().error(response.message)
                return response

        self._busy = True
        t_total = time.time()
        picked = 0
        failed = 0

        try:
            self._publish_status(f'Cluster {tree_id}: {len(bolls)} bolls — going HOME')
            if not self._go_home():
                response.success = False
                response.message = 'Failed to reach HOME at start'
                return response

            # Make sure the gripper is OPEN before picking the first boll
            # (visual: fingers spread). Idempotent — fine if already open.
            self._gripper_call(self.gripper_open_cli, 'gripper/open')

            for i, boll in enumerate(bolls, 1):
                tag = f'[{i}/{len(bolls)}] {boll["id"]} ({boll["type"]})'
                bx, by, bz = float(boll['x']), float(boll['y']), float(boll['z'])

                # 1. Go to boll
                self._publish_status(f'{tag} → boll @ ({bx:.3f},{by:.3f},{bz:.3f})')
                t_step = time.time()
                if not self._go_to_xyz(bx, by, bz, approach_orientation=True):
                    self.get_logger().warn(f'{tag} reach FAILED — skipping')
                    failed += 1
                    continue
                self.get_logger().info(f'{tag} reached in {time.time()-t_step:.1f}s')

                # 2. Visual GRIP CLOSE — animate fingers in Gazebo (mock,
                #    boll motion is handled by teleport below).
                self.get_logger().info(f'{tag} [GRIP CLOSE] commanding /gripper/close')
                self._gripper_call(self.gripper_close_cli, 'gripper/close')
                close_dt = float(self.get_parameter('mock_close_delay_s').value)
                time.sleep(close_dt)  # extra dwell so the close is visible

                # 3. Teleport boll → TCP and START continuous follow
                if not self._teleport_to_tcp(boll['id']):
                    self.get_logger().warn(f'{tag} teleport-to-TCP failed (continuing)')
                self._carry_start(boll['id'])

                # 4. Heuristic reach to reservoir hover: HOME → joint1 rotate → IK.
                #    Carry thread keeps boll glued to TCP through all three stages.
                self._publish_status(f'{tag} → reservoir hover (boll follows)')
                t_step = time.time()
                if not self._go_reservoir():
                    self.get_logger().warn(f'{tag} reservoir reach FAILED')
                    self._carry_stop()
                    failed += 1
                    continue
                self.get_logger().info(
                    f'{tag} reservoir hover reached in {time.time()-t_step:.1f}s')

                # 5. Stop carry → teleport boll into the bin IMMEDIATELY
                #    → only then trigger the visual gripper open. Old
                #    order was carry_stop → open → sleep(0.5s) → teleport,
                #    which left the boll dangling at the last TCP pose
                #    (the hover point in mid-air) while the gripper
                #    animation played — visually "boll detached from
                #    hand, floating in space" for half a second before
                #    snapping into the bin. Doing the teleport first
                #    drops the boll instantly the moment the arm
                #    arrives, and the gripper animation just plays in
                #    parallel against an empty hand.
                self._carry_stop()
                if not self._teleport_to_reservoir(boll['id']):
                    self.get_logger().warn(f'{tag} teleport-to-reservoir failed')
                self.get_logger().info(f'{tag} [GRIP OPEN] commanding /gripper/open')
                self._gripper_call(self.gripper_open_cli, 'gripper/open')
                open_dt = float(self.get_parameter('mock_open_delay_s').value)
                time.sleep(open_dt)  # animation dwell — boll already in bin
                picked += 1

            # Return home
            self._publish_status(f'Cluster {tree_id} done — going HOME')
            self._go_home()

            elapsed = time.time() - t_total
            summary = (f'Cluster {tree_id}: picked {picked}/{len(bolls)} '
                       f'(failed {failed}) in {elapsed:.1f}s')
            self._publish_status(summary)
            response.success = (picked > 0)
            response.message = summary

        except Exception as e:
            self.get_logger().error(f'Harvest crashed: {e}')
            response.success = False
            response.message = f'Crash: {e}'
        finally:
            # Always stop the carry thread on exit, even on crash
            self._carry_stop()
            self._busy = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimpleClusterHarvester()
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
