#!/usr/bin/env python3
"""
RoboCot Control Panel — a PyQt5 GUI to drive the cotton-harvesting demo.

One window to launch the sim, the arm, and the harvest pipeline; to teleop
the base and the wrist camera; to fire the scan / full-run triggers; and to
watch a live camera feed + telemetry — instead of juggling six terminals.

What it gives you
-----------------
  • Camera feed     — rqt_image_view-style viewer with a selectable Image topic
  • Auto-launch     — `ros2 launch robot_arm husky_orchard_demo.launch.py`
                      starts automatically when the app opens
  • Start the car engine — one button runs `row_navigator` AND auto-launches
                      `harvester_modules.launch.py`
  • Start the arm   — `ros2 launch robot_arm_moveit_config moveit.launch.py`
  • WASD teleop     — in-app /cmd_vel publisher (hold the on-screen buttons or
                      press W/A/S/D while the section is focused)
  • Camera teleop   — in-app wrist-camera joint teleop (same key map as
                      orchestrator/arm_teleop, publishes /arm_controller/
                      joint_trajectory)
  • Triggers        — Scan  (/cluster_scan/run)  and
                      Full run (/row_nav/run), both std_srvs/Trigger
  • Telemetry       — latest /row_nav/status, /cluster_harvester/status,
                      /cluster_scan/status, /simple_harvest/status, plus
                      /odom pose, /cmd_vel, and a streaming process-log

Run it (after sourcing your overlay so subprocess `ros2` calls inherit env):
    ros2 run orchestrator control_panel
  or
    python3 control_panel.py

Requires: PyQt5, numpy, rclpy. (cv2 optional — used only for a nicer depth
colormap; falls back to grayscale if missing.)
"""

import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, JointState
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as MsgDuration

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, pyqtSignal

try:
    import cv2  # optional, only for depth colormap
    _HAVE_CV2 = True
    # opencv-python bundles its own (incompatible) Qt plugins and points
    # QT_QPA_PLATFORM_PLUGIN_PATH at them on import — which breaks PyQt5's
    # xcb platform plugin. Drop it so Qt falls back to the system PyQt5 path.
    os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
    os.environ.pop('QT_PLUGIN_PATH', None)
except Exception:
    _HAVE_CV2 = False


# ───────────────────────────── arm teleop constants ─────────────────────────
# Mirrors orchestrator/arm_teleop.py so the in-app controller behaves the same.
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
HOME_JOINTS = [0.0000, -0.922, 2.4494, 0.0, -1.3000, 0.0]
JOINT_LIMITS = [
    (-6.2832, 6.2832), (-6.2832, 6.2832), (-2.7925, 2.7925),
    (-6.2832, 6.2832), (-6.2832, 6.2832), (-6.2832, 6.2832),
]
ARM_PUBLISH_RATE_HZ = 20.0
ARM_TRAJ_HORIZON_S = 0.2

# Base teleop defaults (mirrors wasd_teleop.py)
BASE_LIN = 0.5
BASE_ANG = 1.0


# ─────────────────────────── ROS image → RGB numpy ──────────────────────────
def image_msg_to_rgb(msg: Image):
    """Convert a sensor_msgs/Image to a contiguous HxWx3 uint8 RGB array.

    Handles the common encodings; depth (32FC1/16UC1/mono16) is normalized and
    colormapped (cv2 jet if available, grayscale otherwise).
    """
    h, w, enc = msg.height, msg.width, msg.encoding
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if enc in ('rgb8', 'bgr8'):
        img = buf[:h * msg.step].reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
        if enc == 'bgr8':
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)

    if enc in ('rgba8', 'bgra8'):
        img = buf[:h * msg.step].reshape(h, msg.step)[:, :w * 4].reshape(h, w, 4)
        img = img[:, :, :3]
        if enc == 'bgra8':
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)

    if enc == 'mono8':
        img = buf[:h * msg.step].reshape(h, msg.step)[:, :w]
        return np.ascontiguousarray(np.dstack([img, img, img]))

    if enc in ('32FC1', '16UC1', 'mono16'):
        if enc == '32FC1':
            d = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w).copy()
        else:
            d = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w).astype(np.float32)
        finite = np.isfinite(d) & (d > 0)
        if finite.any():
            lo = float(np.percentile(d[finite], 2))
            hi = float(np.percentile(d[finite], 98))
        else:
            lo, hi = 0.0, 1.0
        if hi <= lo:
            hi = lo + 1.0
        norm = np.clip((d - lo) / (hi - lo), 0.0, 1.0)
        norm[~finite] = 0.0
        g = (norm * 255).astype(np.uint8)
        if _HAVE_CV2:
            color = cv2.applyColorMap(g, cv2.COLORMAP_JET)  # BGR
            return np.ascontiguousarray(color[:, :, ::-1])
        return np.ascontiguousarray(np.dstack([g, g, g]))

    # Unknown encoding — show a flat gray frame rather than crash.
    return np.full((max(h, 1), max(w, 1), 3), 64, dtype=np.uint8)


# ───────────────────────────────── ROS bridge ───────────────────────────────
class RosBridge(QtCore.QObject):
    """rclpy node wrapped in a QObject; ROS callbacks emit Qt signals so the
    GUI thread can update widgets safely (queued cross-thread delivery)."""

    imageReceived = pyqtSignal(object)          # RGB numpy array
    statusReceived = pyqtSignal(str, str)       # (topic, text)
    cmdVelReceived = pyqtSignal(float, float)   # (lin.x, ang.z)
    odomReceived = pyqtSignal(float, float, float)  # (x, y, yaw_deg)
    serviceResult = pyqtSignal(str, bool, str)  # (label, success, message)

    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('control_panel')

        self._sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1)

        self._image_sub = None
        self._image_topic = ''

        # Teleop publishers
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.node.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Telemetry subscriptions
        for topic in ('/row_nav/status', '/cluster_harvester/status',
                      '/cluster_scan/status', '/simple_harvest/status'):
            self.node.create_subscription(
                String, topic,
                lambda m, t=topic: self.statusReceived.emit(t, m.data), 10)

        self.node.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.node.create_subscription(
            Odometry, '/odom', self._on_odom, self._sensor_qos)

        # Latest joint states (for seeding arm teleop)
        self._latest_joints = None
        self._joints_lock = threading.Lock()
        self.node.create_subscription(
            JointState, '/joint_states', self._on_joints, 10)

        # Service clients for triggers
        self.scan_cli = self.node.create_client(Trigger, '/cluster_scan/run')
        self.run_cli = self.node.create_client(Trigger, '/row_nav/run')

        self._spin = True
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    # ---- spinning ----
    def _spin_loop(self):
        while self._spin and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def shutdown(self):
        self._spin = False
        try:
            self._thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass

    # ---- callbacks ----
    def _on_cmd_vel(self, msg: Twist):
        self.cmdVelReceived.emit(msg.linear.x, msg.angular.z)

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.degrees(np.arctan2(siny, cosy))
        p = msg.pose.pose.position
        self.odomReceived.emit(p.x, p.y, float(yaw))

    def _on_joints(self, msg: JointState):
        d = dict(zip(msg.name, msg.position))
        if all(n in d for n in JOINT_NAMES):
            with self._joints_lock:
                self._latest_joints = [float(d[n]) for n in JOINT_NAMES]

    def latest_joints(self):
        with self._joints_lock:
            return None if self._latest_joints is None else list(self._latest_joints)

    # ---- image topic switching ----
    def set_image_topic(self, topic: str):
        if topic == self._image_topic and self._image_sub is not None:
            return
        if self._image_sub is not None:
            self.node.destroy_subscription(self._image_sub)
            self._image_sub = None
        self._image_topic = topic
        if topic:
            self._image_sub = self.node.create_subscription(
                Image, topic, self._on_image, self._sensor_qos)

    def _on_image(self, msg: Image):
        try:
            rgb = image_msg_to_rgb(msg)
            self.imageReceived.emit(rgb)
        except Exception as e:
            self.node.get_logger().warn(f'image convert failed: {e}')

    def list_image_topics(self):
        topics = self.node.get_topic_names_and_types()
        out = []
        for name, types in topics:
            if any('sensor_msgs/msg/Image' == t for t in types):
                out.append(name)
        return sorted(out)

    # ---- teleop publishing ----
    def publish_cmd_vel(self, lin, ang):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.cmd_pub.publish(msg)

    def publish_arm_traj(self, positions):
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = MsgDuration(
            sec=int(ARM_TRAJ_HORIZON_S),
            nanosec=int((ARM_TRAJ_HORIZON_S % 1) * 1e9))
        traj.points = [pt]
        self.arm_pub.publish(traj)

    # ---- triggers (async, result via signal) ----
    def call_trigger(self, which: str, label: str):
        cli = self.scan_cli if which == 'scan' else self.run_cli
        if not cli.service_is_ready():
            if not cli.wait_for_service(timeout_sec=2.0):
                self.serviceResult.emit(label, False, 'service not available')
                return
        fut = cli.call_async(Trigger.Request())

        def _done(f):
            try:
                res = f.result()
                self.serviceResult.emit(label, bool(res.success), str(res.message))
            except Exception as e:
                self.serviceResult.emit(label, False, str(e))

        fut.add_done_callback(_done)


# ───────────────────────────── process manager ──────────────────────────────
class ProcessManager(QtCore.QObject):
    """Runs `ros2 ...` commands as child process groups, streams their output
    as Qt signals, and tears them all down on exit."""

    logLine = pyqtSignal(str, str)     # (tag, line)
    stateChanged = pyqtSignal(str, bool)  # (key, running)

    def __init__(self):
        super().__init__()
        self._procs = {}  # key -> Popen

    def is_running(self, key):
        p = self._procs.get(key)
        return p is not None and p.poll() is None

    def start(self, key, argv, tag=None):
        if self.is_running(key):
            return
        tag = tag or key
        env = dict(os.environ)
        env.setdefault('PYTHONUNBUFFERED', '1')
        try:
            proc = subprocess.Popen(
                argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, env=env,
                start_new_session=True)  # own process group → clean kill
        except Exception as e:
            self.logLine.emit(tag, f'[failed to start] {e}')
            return
        self._procs[key] = proc
        self.stateChanged.emit(key, True)
        self.logLine.emit(tag, f'$ {" ".join(argv)}  (pid {proc.pid})')
        threading.Thread(target=self._reader, args=(key, tag, proc),
                         daemon=True).start()

    def _reader(self, key, tag, proc):
        try:
            for line in iter(proc.stdout.readline, ''):
                if line:
                    self.logLine.emit(tag, line.rstrip('\n'))
        except Exception:
            pass
        proc.wait()
        self.logLine.emit(tag, f'[exited rc={proc.returncode}]')
        self.stateChanged.emit(key, False)

    def stop(self, key):
        proc = self._procs.get(key)
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass
        # give it a moment, then hard-kill
        def _ensure_dead(p=proc):
            for _ in range(30):
                if p.poll() is not None:
                    return
                time.sleep(0.1)
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        threading.Thread(target=_ensure_dead, daemon=True).start()

    def stop_all(self):
        for key in list(self._procs):
            self.stop(key)


# ─────────────────────────────── camera widget ──────────────────────────────
class CameraView(QtWidgets.QLabel):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(480, 360)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet(
            'background:#0b1120;color:#64748b;border:1px solid #334155;'
            'border-radius:10px;font-size:14px;')
        self.setText('waiting for image…')
        self._pix = None

    def set_frame(self, rgb: np.ndarray):
        h, w, _ = rgb.shape
        qimg = QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888)
        self._pix = QtGui.QPixmap.fromImage(qimg.copy())
        self._rescale()

    def _rescale(self):
        if self._pix is not None:
            self.setPixmap(self._pix.scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, e):
        self._rescale()
        super().resizeEvent(e)


# ───────────────────────── press-and-hold button ────────────────────────────
class HoldButton(QtWidgets.QPushButton):
    """Emits `held(True)` on press and `held(False)` on release — for teleop."""
    held = pyqtSignal(bool)

    def __init__(self, label):
        super().__init__(label)
        self.setFocusPolicy(Qt.NoFocus)
        self.pressed.connect(lambda: self.held.emit(True))
        self.released.connect(lambda: self.held.emit(False))


# ───────────────────────────── main window ──────────────────────────────────
class ControlPanel(QtWidgets.QMainWindow):
    def __init__(self, bridge: RosBridge):
        super().__init__()
        self.bridge = bridge
        self.procs = ProcessManager()
        self.setWindowTitle('RoboCot Control Panel')
        self.resize(1280, 820)

        # base teleop velocity state (republished at 20 Hz)
        self._base_lin = 0.0
        self._base_ang = 0.0
        self._base_lin_scale = BASE_LIN
        self._base_ang_scale = BASE_ANG
        self._base_enabled = False

        # arm teleop state
        self._arm_target = None
        self._arm_vel = [0.0] * 6
        self._arm_speed = 0.6
        self._arm_enabled = False
        self._arm_last_key_t = 0.0

        # camera fps tracking
        self._frame_count = 0
        self._frame_count_prev = 0
        self._last_frame_t = 0.0
        self._last_shape = None

        self._build_ui()
        self._wire_signals()

        # 1 Hz camera-feed status (fps / size / staleness)
        self._cam_timer = QtCore.QTimer(self)
        self._cam_timer.timeout.connect(self._update_cam_status)
        self._cam_timer.start(1000)

        # 20 Hz teleop publisher loop
        self._teleop_timer = QtCore.QTimer(self)
        self._teleop_timer.timeout.connect(self._teleop_tick)
        self._teleop_timer.start(int(1000 / ARM_PUBLISH_RATE_HZ))

        # auto-launch the sim on startup (requirement 2)
        QtCore.QTimer.singleShot(300, self._start_sim)

    # ---------------- UI construction ----------------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(14)

        # ===== LEFT: camera + telemetry =====
        left = QtWidgets.QVBoxLayout()
        left.setSpacing(12)
        root.addLayout(left, 3)

        cam_box = QtWidgets.QGroupBox('Camera feed')
        cam_l = QtWidgets.QVBoxLayout(cam_box)
        topic_row = QtWidgets.QHBoxLayout()
        topic_row.addWidget(QtWidgets.QLabel('Topic:'))
        self.topic_combo = QtWidgets.QComboBox()
        self.topic_combo.setEditable(True)
        self.topic_combo.addItems(
            ['/camera/color/image_raw', '/camera/depth/image_raw'])
        self.refresh_btn = QtWidgets.QPushButton('⟳ Refresh')
        topic_row.addWidget(self.topic_combo, 1)
        topic_row.addWidget(self.refresh_btn)
        self.cam_status = QtWidgets.QLabel('no frames yet')
        self.cam_status.setObjectName('telemVal')
        topic_row.addWidget(self.cam_status)
        cam_l.addLayout(topic_row)
        self.camera_view = CameraView()
        cam_l.addWidget(self.camera_view, 1)
        left.addWidget(cam_box, 1)

        tele_box = QtWidgets.QGroupBox('Telemetry')
        tele_l = QtWidgets.QVBoxLayout(tele_box)
        grid = QtWidgets.QGridLayout()
        self.lbl_odom = QtWidgets.QLabel('—')
        self.lbl_cmd = QtWidgets.QLabel('—')
        self.lbl_rownav = QtWidgets.QLabel('—')
        self.lbl_harv = QtWidgets.QLabel('—')
        self.lbl_scan = QtWidgets.QLabel('—')
        self.lbl_simple = QtWidgets.QLabel('—')
        rows = [
            ('Odom (x,y,yaw°)', self.lbl_odom),
            ('cmd_vel (lin,ang)', self.lbl_cmd),
            ('row_nav', self.lbl_rownav),
            ('cluster_harvester', self.lbl_harv),
            ('cluster_scan', self.lbl_scan),
            ('simple_harvest', self.lbl_simple),
        ]
        for i, (name, lbl) in enumerate(rows):
            lbl.setObjectName('telemVal')
            lbl.setWordWrap(True)
            key = QtWidgets.QLabel(name)
            key.setObjectName('telemKey')
            grid.addWidget(key, i, 0, Qt.AlignTop)
            grid.addWidget(lbl, i, 1)
        grid.setColumnStretch(1, 1)
        grid.setVerticalSpacing(7)
        tele_l.addLayout(grid)
        tele_l.addWidget(QtWidgets.QLabel('Process log:'))
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        self.log.setStyleSheet(
            'font-family:monospace;font-size:11px;background:#0c0c0c;color:#cfcfcf;')
        tele_l.addWidget(self.log, 1)
        left.addWidget(tele_box, 1)

        # ===== RIGHT: controls =====
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(12)
        root.addLayout(right, 2)

        # -- launch / process controls --
        launch_box = QtWidgets.QGroupBox('Launch / Pipeline')
        lb = QtWidgets.QVBoxLayout(launch_box)
        self.sim_btn = QtWidgets.QPushButton('Sim: husky_orchard_demo (auto)')
        self.sim_btn.setObjectName('primaryBtn')
        self.arm_launch_btn = QtWidgets.QPushButton('🦾 Start the arm  (MoveIt)')
        self.arm_launch_btn.setObjectName('infoBtn')
        self.engine_btn = QtWidgets.QPushButton('🚜 Start the car engine')
        self.engine_btn.setObjectName('successBtn')
        for b in (self.sim_btn, self.arm_launch_btn, self.engine_btn):
            b.setMinimumHeight(44)
            lb.addWidget(b)
        lb.addWidget(QtWidgets.QLabel(
            '<i>“car engine” = row_navigator + harvester_modules</i>'))
        right.addWidget(launch_box)

        # -- triggers --
        trig_box = QtWidgets.QGroupBox('Triggers')
        tb = QtWidgets.QHBoxLayout(trig_box)
        self.scan_btn = QtWidgets.QPushButton('🔍 Scan')
        self.scan_btn.setObjectName('accentBtn')
        self.fullrun_btn = QtWidgets.QPushButton('▶ Full run')
        self.fullrun_btn.setObjectName('successBtn')
        for b in (self.scan_btn, self.fullrun_btn):
            b.setMinimumHeight(48)
            tb.addWidget(b)
        right.addWidget(trig_box)

        # -- base (WASD) teleop --
        self.base_box = QtWidgets.QGroupBox('Base teleop (WASD)')
        bb = QtWidgets.QVBoxLayout(self.base_box)
        self.base_toggle = QtWidgets.QPushButton('Start WASD teleop')
        self.base_toggle.setObjectName('toggleBtn')
        self.base_toggle.setCheckable(True)
        bb.addWidget(self.base_toggle)
        pad = QtWidgets.QGridLayout()
        pad.setSpacing(8)
        self.btn_w = HoldButton('W\n↑ fwd')
        self.btn_a = HoldButton('A\n↶ left')
        self.btn_s = HoldButton('S\n↓ back')
        self.btn_d = HoldButton('D\n↷ right')
        self.btn_stop = QtWidgets.QPushButton('STOP')
        self.btn_stop.setObjectName('dangerBtn')
        for b in (self.btn_w, self.btn_a, self.btn_s, self.btn_d):
            b.setObjectName('padBtn')
            b.setMinimumHeight(54)
        self.btn_stop.setMinimumHeight(54)
        pad.addWidget(self.btn_w, 0, 1)
        pad.addWidget(self.btn_a, 1, 0)
        pad.addWidget(self.btn_stop, 1, 1)
        pad.addWidget(self.btn_d, 1, 2)
        pad.addWidget(self.btn_s, 2, 1)
        bb.addLayout(pad)
        speed_row = QtWidgets.QHBoxLayout()
        speed_row.addWidget(QtWidgets.QLabel('lin'))
        self.base_lin_spin = QtWidgets.QDoubleSpinBox()
        self.base_lin_spin.setRange(0.05, 2.0)
        self.base_lin_spin.setSingleStep(0.05)
        self.base_lin_spin.setValue(BASE_LIN)
        speed_row.addWidget(self.base_lin_spin)
        speed_row.addWidget(QtWidgets.QLabel('ang'))
        self.base_ang_spin = QtWidgets.QDoubleSpinBox()
        self.base_ang_spin.setRange(0.05, 3.0)
        self.base_ang_spin.setSingleStep(0.05)
        self.base_ang_spin.setValue(BASE_ANG)
        speed_row.addWidget(self.base_ang_spin)
        bb.addLayout(speed_row)
        self.base_hint = QtWidgets.QLabel(
            'Enable, then hold buttons — or focus this box and press W/A/S/D.')
        self.base_hint.setWordWrap(True)
        bb.addWidget(self.base_hint)
        self.base_box.setFocusPolicy(Qt.StrongFocus)
        right.addWidget(self.base_box)

        # -- camera/arm teleop --
        self.arm_box = QtWidgets.QGroupBox('Camera teleop (wrist arm)')
        ab = QtWidgets.QVBoxLayout(self.arm_box)
        self.arm_toggle = QtWidgets.QPushButton('Start camera teleop')
        self.arm_toggle.setObjectName('toggleBtn')
        self.arm_toggle.setCheckable(True)
        ab.addWidget(self.arm_toggle)
        agrid = QtWidgets.QGridLayout()
        agrid.setSpacing(8)
        # (label, joint idx, sign)
        self.arm_btns = []

        def mk(label, idx, sign, r, c):
            b = HoldButton(label)
            b.setObjectName('padBtn')
            b.setMinimumHeight(42)
            b.held.connect(lambda on, i=idx, s=sign: self._arm_hold(i, s, on))
            agrid.addWidget(b, r, c)
            self.arm_btns.append(b)
            return b

        mk('PAN ◀ (a)', 0, +1.0, 0, 0)
        mk('PAN ▶ (d)', 0, -1.0, 0, 1)
        mk('TILT ▲ (w)', 4, +1.0, 1, 0)
        mk('TILT ▼ (s)', 4, -1.0, 1, 1)
        mk('HIGHER (e)', 1, +1.0, 2, 0)
        mk('LOWER (q)', 1, -1.0, 2, 1)
        mk('EXTEND (r)', 2, +1.0, 3, 0)
        mk('RETRACT (f)', 2, -1.0, 3, 1)
        mk('ROLL + (z)', 5, +1.0, 4, 0)
        mk('ROLL − (x)', 5, -1.0, 4, 1)
        ab.addLayout(agrid)
        arow = QtWidgets.QHBoxLayout()
        self.arm_home_btn = QtWidgets.QPushButton('HOME (h)')
        self.arm_stop_btn = QtWidgets.QPushButton('STOP')
        self.arm_stop_btn.setObjectName('dangerBtn')
        arow.addWidget(self.arm_home_btn)
        arow.addWidget(self.arm_stop_btn)
        arow.addWidget(QtWidgets.QLabel('speed'))
        self.arm_speed_spin = QtWidgets.QDoubleSpinBox()
        self.arm_speed_spin.setRange(0.05, 2.0)
        self.arm_speed_spin.setSingleStep(0.05)
        self.arm_speed_spin.setValue(0.6)
        arow.addWidget(self.arm_speed_spin)
        ab.addLayout(arow)
        self.arm_hint = QtWidgets.QLabel(
            'Enable, then hold buttons — or focus this box and use the key in '
            'each label (a/d/w/s/e/q/r/f/z/x, h=home, space=stop).')
        self.arm_hint.setWordWrap(True)
        ab.addWidget(self.arm_hint)
        self.arm_box.setFocusPolicy(Qt.StrongFocus)
        right.addWidget(self.arm_box)

        right.addStretch(1)

    # ---------------- signal wiring ----------------
    def _wire_signals(self):
        # ROS → GUI
        self.bridge.imageReceived.connect(self.camera_view.set_frame)
        self.bridge.imageReceived.connect(self._on_frame_meta)
        self.bridge.cmdVelReceived.connect(
            lambda l, a: self.lbl_cmd.setText(f'lin={l:+.2f}  ang={a:+.2f}'))
        self.bridge.odomReceived.connect(
            lambda x, y, yaw: self.lbl_odom.setText(
                f'x={x:+.2f}  y={y:+.2f}  yaw={yaw:+.1f}°'))
        self.bridge.statusReceived.connect(self._on_status)
        self.bridge.serviceResult.connect(self._on_service_result)

        # process state → GUI
        self.procs.logLine.connect(self._append_log)
        self.procs.stateChanged.connect(self._on_proc_state)

        # camera topic
        self.topic_combo.currentTextChanged.connect(self.bridge.set_image_topic)
        self.refresh_btn.clicked.connect(self._refresh_topics)
        self.bridge.set_image_topic(self.topic_combo.currentText())

        # launch buttons
        self.sim_btn.clicked.connect(self._toggle_sim)
        self.arm_launch_btn.clicked.connect(self._toggle_arm_launch)
        self.engine_btn.clicked.connect(self._toggle_engine)

        # triggers
        self.scan_btn.clicked.connect(
            lambda: self.bridge.call_trigger('scan', 'Scan'))
        self.fullrun_btn.clicked.connect(
            lambda: self.bridge.call_trigger('run', 'Full run'))

        # base teleop
        self.base_toggle.toggled.connect(self._toggle_base)
        self.btn_w.held.connect(lambda on: self._base_hold(1.0, 0.0, on))
        self.btn_s.held.connect(lambda on: self._base_hold(-1.0, 0.0, on))
        self.btn_a.held.connect(lambda on: self._base_hold(0.0, 1.0, on))
        self.btn_d.held.connect(lambda on: self._base_hold(0.0, -1.0, on))
        self.btn_stop.clicked.connect(lambda: self._base_set(0.0, 0.0))
        self.base_lin_spin.valueChanged.connect(
            lambda v: setattr(self, '_base_lin_scale', v))
        self.base_ang_spin.valueChanged.connect(
            lambda v: setattr(self, '_base_ang_scale', v))

        # arm teleop
        self.arm_toggle.toggled.connect(self._toggle_arm)
        self.arm_home_btn.clicked.connect(self._arm_home)
        self.arm_stop_btn.clicked.connect(self._arm_stop)
        self.arm_speed_spin.valueChanged.connect(
            lambda v: setattr(self, '_arm_speed', v))

    # ---------------- camera feed status ----------------
    def _on_frame_meta(self, rgb):
        self._frame_count += 1
        self._last_frame_t = time.time()
        self._last_shape = rgb.shape

    def _update_cam_status(self):
        fps = self._frame_count - self._frame_count_prev
        self._frame_count_prev = self._frame_count
        age = time.time() - self._last_frame_t if self._last_frame_t else 1e9
        if self._frame_count == 0:
            self.cam_status.setText('⏳ no frames yet')
            self.cam_status.setStyleSheet('color:#f59e0b;')
        elif age > 2.0:
            self.cam_status.setText(f'⚠ stale ({age:.0f}s)')
            self.cam_status.setStyleSheet('color:#ef4444;')
        else:
            h, w = self._last_shape[0], self._last_shape[1]
            self.cam_status.setText(f'● {w}×{h} @ {fps} fps')
            self.cam_status.setStyleSheet('color:#22c55e;')

    # ---------------- camera topics ----------------
    def _refresh_topics(self):
        current = self.topic_combo.currentText()
        topics = self.bridge.list_image_topics()
        self.topic_combo.blockSignals(True)
        self.topic_combo.clear()
        if not topics:
            topics = ['/camera/color/image_raw', '/camera/depth/image_raw']
        self.topic_combo.addItems(topics)
        if current in topics:
            self.topic_combo.setCurrentText(current)
        self.topic_combo.blockSignals(False)
        self._append_log('panel', f'image topics: {", ".join(topics)}')

    # ---------------- launch handlers ----------------
    def _start_sim(self):
        self.procs.start(
            'sim',
            ['ros2', 'launch', 'robot_arm', 'husky_orchard_demo.launch.py'],
            tag='sim')

    def _toggle_sim(self):
        if self.procs.is_running('sim'):
            self.procs.stop('sim')
        else:
            self._start_sim()

    def _toggle_arm_launch(self):
        if self.procs.is_running('arm'):
            self.procs.stop('arm')
        else:
            self.procs.start(
                'arm',
                ['ros2', 'launch', 'robot_arm_moveit_config', 'moveit.launch.py'],
                tag='arm')

    def _toggle_engine(self):
        # "car engine" = row_navigator + harvester_modules together
        running = self.procs.is_running('rownav') or self.procs.is_running('harvester')
        if running:
            self.procs.stop('rownav')
            self.procs.stop('harvester')
        else:
            self.procs.start(
                'harvester',
                ['ros2', 'launch', 'orchestrator', 'harvester_modules.launch.py'],
                tag='harvester')
            self.procs.start(
                'rownav',
                ['ros2', 'run', 'orchestrator', 'row_navigator'],
                tag='rownav')

    @staticmethod
    def _set_running(btn, running):
        """Toggle the `running` dynamic property and re-polish so the QSS
        running-state (red 'click to stop') styling takes effect live."""
        btn.setProperty('running', bool(running))
        btn.style().unpolish(btn)
        btn.style().polish(btn)

    def _on_proc_state(self, key, running):
        if key == 'sim':
            self.sim_btn.setText(
                '🟢 Sim running — click to stop' if running
                else 'Sim: husky_orchard_demo (start)')
            self._set_running(self.sim_btn, running)
        elif key == 'arm':
            self.arm_launch_btn.setText(
                '🛑 Stop the arm (MoveIt)' if running
                else '🦾 Start the arm  (MoveIt)')
            self._set_running(self.arm_launch_btn, running)
        elif key in ('rownav', 'harvester'):
            eng = self.procs.is_running('rownav') or self.procs.is_running('harvester')
            self.engine_btn.setText(
                '🛑 Stop the car engine' if eng else '🚜 Start the car engine')
            self._set_running(self.engine_btn, eng)

    # ---------------- telemetry ----------------
    def _on_status(self, topic, text):
        m = {
            '/row_nav/status': self.lbl_rownav,
            '/cluster_harvester/status': self.lbl_harv,
            '/cluster_scan/status': self.lbl_scan,
            '/simple_harvest/status': self.lbl_simple,
        }
        if topic in m:
            m[topic].setText(text)
        self._append_log(topic.strip('/').split('/')[0], text)

    def _on_service_result(self, label, success, message):
        tag = 'OK' if success else 'FAIL'
        self._append_log('trigger', f'[{label}] {tag}: {message}')

    def _append_log(self, tag, line):
        self.log.appendPlainText(f'[{tag}] {line}')

    # ---------------- base teleop ----------------
    def _toggle_base(self, on):
        self._base_enabled = on
        self.base_toggle.setText('Stop WASD teleop' if on else 'Start WASD teleop')
        if on:
            self.base_box.setFocus()
        else:
            self._base_set(0.0, 0.0)

    def _base_hold(self, lin_dir, ang_dir, on):
        if not self._base_enabled:
            return
        if on:
            self._base_lin = lin_dir * self._base_lin_scale
            self._base_ang = ang_dir * self._base_ang_scale
        else:
            self._base_lin = 0.0
            self._base_ang = 0.0

    def _base_set(self, lin, ang):
        self._base_lin = lin
        self._base_ang = ang
        self.bridge.publish_cmd_vel(lin, ang)

    # ---------------- arm teleop ----------------
    def _toggle_arm(self, on):
        self._arm_enabled = on
        self.arm_toggle.setText('Stop camera teleop' if on else 'Start camera teleop')
        if on:
            seed = self.bridge.latest_joints()
            self._arm_target = seed if seed is not None else list(HOME_JOINTS)
            self.arm_box.setFocus()
            self._append_log('arm_teleop',
                             'enabled — target seeded from /joint_states'
                             if seed is not None else
                             'enabled — no /joint_states yet, seeded HOME')
        else:
            self._arm_vel = [0.0] * 6

    def _arm_hold(self, idx, sign, on):
        if not self._arm_enabled:
            return
        self._arm_vel = [0.0] * 6
        if on:
            self._arm_vel[idx] = sign * self._arm_speed
            self._arm_last_key_t = time.time()

    def _arm_home(self):
        if not self._arm_enabled:
            return
        self._arm_target = list(HOME_JOINTS)
        self._arm_vel = [0.0] * 6

    def _arm_stop(self):
        self._arm_vel = [0.0] * 6

    # ---------------- 20 Hz teleop tick ----------------
    def _teleop_tick(self):
        # base: republish current velocity so the robot keeps moving
        if self._base_enabled:
            self.bridge.publish_cmd_vel(self._base_lin, self._base_ang)

        # arm: integrate target and publish
        if self._arm_enabled and self._arm_target is not None:
            dt = 1.0 / ARM_PUBLISH_RATE_HZ
            if any(self._arm_vel):
                for i in range(6):
                    self._arm_target[i] += self._arm_vel[i] * dt
                    lo, hi = JOINT_LIMITS[i]
                    self._arm_target[i] = max(lo, min(hi, self._arm_target[i]))
            self.bridge.publish_arm_traj(self._arm_target)

    # ---------------- keyboard teleop ----------------
    def keyPressEvent(self, e):
        if e.isAutoRepeat():
            return
        k = e.text().lower()
        focus = self.focusWidget()
        in_base = self.base_box.isAncestorOf(focus) or focus is self.base_box
        in_arm = self.arm_box.isAncestorOf(focus) or focus is self.arm_box

        if self._base_enabled and in_base:
            m = {'w': (1.0, 0.0), 's': (-1.0, 0.0),
                 'a': (0.0, 1.0), 'd': (0.0, -1.0),
                 'q': (1.0, 1.0), 'e': (1.0, -1.0),
                 'z': (-1.0, 1.0), 'c': (-1.0, -1.0)}
            if k in m:
                self._base_hold(m[k][0], m[k][1], True)
                return
            if k in (' ', 'x'):
                self._base_set(0.0, 0.0)
                return

        if self._arm_enabled and in_arm:
            m = {'a': (0, +1.0), 'd': (0, -1.0),
                 'e': (1, +1.0), 'q': (1, -1.0),
                 'r': (2, +1.0), 'f': (2, -1.0),
                 'w': (4, +1.0), 's': (4, -1.0),
                 'z': (5, +1.0), 'x': (5, -1.0)}
            if k in m:
                self._arm_hold(m[k][0], m[k][1], True)
                return
            if k == 'h':
                self._arm_home()
                return
            if k == ' ':
                self._arm_stop()
                return
        super().keyPressEvent(e)

    def keyReleaseEvent(self, e):
        if e.isAutoRepeat():
            return
        # stop motion on release for held keys
        if self._base_enabled:
            self._base_lin = 0.0
            self._base_ang = 0.0
        if self._arm_enabled:
            self._arm_vel = [0.0] * 6
        super().keyReleaseEvent(e)

    # ---------------- shutdown ----------------
    def closeEvent(self, e):
        try:
            self.bridge.publish_cmd_vel(0.0, 0.0)
        except Exception:
            pass
        self.procs.stop_all()
        self.bridge.shutdown()
        super().closeEvent(e)


THEME_QSS = """
* { font-family: 'Segoe UI', 'Ubuntu', 'Noto Sans', sans-serif; font-size: 13px; }
QMainWindow, QWidget { background: #0f172a; color: #e2e8f0; }

QGroupBox {
    background: #1e293b;
    border: 1px solid #334155;
    border-radius: 12px;
    margin-top: 16px;
    padding: 16px 12px 12px 12px;
    font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 14px; padding: 2px 10px;
    color: #38bdf8;
    background: #1e293b;
    border-radius: 6px;
}

QLabel { color: #e2e8f0; background: transparent; }
QLabel#telemKey { color: #94a3b8; font-weight: 600; }
QLabel#telemVal { color: #38bdf8; font-family: 'JetBrains Mono','Consolas',monospace; }

/* ---- base buttons ---- */
QPushButton {
    background: #334155;
    color: #e2e8f0;
    border: 1px solid #475569;
    border-radius: 9px;
    padding: 9px 14px;
    font-weight: 600;
}
QPushButton:hover { background: #3d4d63; border-color: #64748b; }
QPushButton:pressed { background: #475569; }
QPushButton:disabled { color: #64748b; background: #18202f; border-color: #2b3648; }

/* ---- pad (teleop) buttons ---- */
QPushButton#padBtn {
    background: #273449; border: 1px solid #3b4a63; font-weight: 700; font-size: 14px;
}
QPushButton#padBtn:hover { background: #324465; border-color: #6366f1; color: #c7d2fe; }
QPushButton#padBtn:pressed { background: #4338ca; border-color: #818cf8; color: white; }

/* ---- accent / semantic buttons ---- */
QPushButton#primaryBtn { background: #2563eb; border-color: #3b82f6; color: white; }
QPushButton#primaryBtn:hover { background: #1d4ed8; }
QPushButton#infoBtn { background: #0891b2; border-color: #06b6d4; color: white; }
QPushButton#infoBtn:hover { background: #0e7490; }
QPushButton#accentBtn { background: #7c3aed; border-color: #8b5cf6; color: white; }
QPushButton#accentBtn:hover { background: #6d28d9; }
QPushButton#successBtn { background: #16a34a; border-color: #22c55e; color: white; }
QPushButton#successBtn:hover { background: #15803d; }
QPushButton#dangerBtn { background: #b91c1c; border-color: #ef4444; color: white; }
QPushButton#dangerBtn:hover { background: #991b1b; }

/* running = process active -> red 'click to stop' for any semantic button */
QPushButton#primaryBtn[running="true"],
QPushButton#infoBtn[running="true"],
QPushButton#successBtn[running="true"] {
    background: #dc2626; border-color: #f87171; color: white;
}
QPushButton#primaryBtn[running="true"]:hover,
QPushButton#infoBtn[running="true"]:hover,
QPushButton#successBtn[running="true"]:hover { background: #b91c1c; }

/* toggles (WASD / camera teleop on-off) */
QPushButton#toggleBtn { background: #334155; border: 1px solid #475569; }
QPushButton#toggleBtn:checked { background: #16a34a; border-color: #22c55e; color: white; }
QPushButton#toggleBtn:checked:hover { background: #15803d; }

/* ---- inputs ---- */
QComboBox, QDoubleSpinBox {
    background: #0b1120; border: 1px solid #475569; border-radius: 7px;
    padding: 6px 8px; color: #e2e8f0; selection-background-color: #6366f1;
}
QComboBox:hover, QDoubleSpinBox:hover { border-color: #64748b; }
QComboBox::drop-down { border: none; width: 22px; }
QComboBox QAbstractItemView {
    background: #1e293b; border: 1px solid #475569;
    selection-background-color: #6366f1; color: #e2e8f0; outline: none;
}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
    background: #273449; border: none; width: 16px;
}

/* ---- log / scrollbars ---- */
QPlainTextEdit {
    background: #0b1120; border: 1px solid #334155; border-radius: 9px;
    font-family: 'JetBrains Mono','Consolas',monospace; font-size: 11px;
    color: #cbd5e1; padding: 6px;
}
QScrollBar:vertical { background: transparent; width: 11px; margin: 2px; }
QScrollBar::handle:vertical { background: #475569; border-radius: 5px; min-height: 24px; }
QScrollBar::handle:vertical:hover { background: #64748b; }
QScrollBar::add-line, QScrollBar::sub-line { height: 0; }
QScrollBar:horizontal { background: transparent; height: 11px; margin: 2px; }
QScrollBar::handle:horizontal { background: #475569; border-radius: 5px; min-width: 24px; }
"""


def main():
    # Create the QApplication before any QObject (RosBridge) to avoid
    # "QObject::moveToThread" warnings and ensure the platform plugin loads.
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setStyleSheet(THEME_QSS)
    rclpy.init()
    bridge = RosBridge()
    win = ControlPanel(bridge)
    win.show()
    try:
        rc = app.exec_()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(rc)


if __name__ == '__main__':
    main()
