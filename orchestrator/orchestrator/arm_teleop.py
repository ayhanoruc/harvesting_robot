#!/usr/bin/env python3
"""
WASD-style arm teleop for the wrist camera.

Lets you steer the TCP (= wrist camera) around interactively from a terminal.
Each keypress sets a target joint-velocity; the node integrates the target
joint positions and republishes JointTrajectory at 20 Hz directly to
/arm_controller/joint_trajectory (bypassing MoveGroup planning — same
direct-trajectory mechanism as arm_commander.send_joint_goal_direct).

Subscribes /joint_states to initialize the target from the current pose,
then integrates with URDF joint limit clamping. Auto-stops after a short
inactivity window so the arm doesn't drift if you stop pressing keys.

Key bindings (rad/s scaled by `speed`):
    a / d    PAN  ±        (joint1, base rotate around Z)
    w / s    TILT up/down  (joint5, wrist pitch)
    e / q    HIGHER/LOWER  (joint2, shoulder pitch)
    r / f    EXTEND/RETRACT (joint3, elbow extend)
    z / x    ROLL ±        (joint6, wrist roll)
    SPACE    STOP          (zero all velocities)
    h        HOME          (reset target to HOME_JOINTS)
    + / -    speed scale up/down
    CTRL-C   quit

Note: While this teleop is running, do NOT call /go_to_pose or
/go_to_named on arm_commander at the same time — both publish to the
same controller and will fight. Use one at a time.

Usage:
    ros2 run orchestrator arm_teleop
"""

import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

HOME_JOINTS = [0.0000, -0.922, 2.4494, 0.0, -1.3000, 0.0]

# M1013 URDF position limits (radians). Same as arm_commander.JOINT_LIMITS.
JOINT_LIMITS = [
    (-6.2832, 6.2832),   # joint1
    (-6.2832, 6.2832),   # joint2
    (-2.7925, 2.7925),   # joint3 — restricted
    (-6.2832, 6.2832),   # joint4
    (-6.2832, 6.2832),   # joint5
    (-6.2832, 6.2832),   # joint6
]

DEFAULT_SPEED = 0.6      # rad/s base velocity scale
SPEED_STEP = 0.1
INACTIVITY_STOP_S = 0.25 # auto-zero velocities after this long with no keypress
PUBLISH_RATE_HZ = 20.0
TRAJECTORY_HORIZON_S = 0.2  # publish each point with this time_from_start

# Key → (joint index, sign)
KEY_BINDINGS = {
    'a': (0, +1.0), 'd': (0, -1.0),
    'e': (1, +1.0), 'q': (1, -1.0),
    'r': (2, +1.0), 'f': (2, -1.0),
    'w': (4, +1.0), 's': (4, -1.0),
    'z': (5, +1.0), 'x': (5, -1.0),
}


def get_key(settings, timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class ArmTeleop(Node):

    def __init__(self):
        super().__init__('arm_teleop')

        self.pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

        self.target = None              # filled once /joint_states arrives
        self.velocities = [0.0] * 6     # rad/s per joint
        self.speed = DEFAULT_SPEED
        self.last_key_t = time.time()
        self.last_tick_t = time.time()

        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

    def _joint_state_cb(self, msg: JointState):
        # Only set target once at startup; from then on we own it.
        if self.target is not None:
            return
        d = dict(zip(msg.name, msg.position))
        if all(n in d for n in JOINT_NAMES):
            self.target = [float(d[n]) for n in JOINT_NAMES]
            self.get_logger().info(
                f'Initial target from /joint_states: '
                f'{[f"{v:.2f}" for v in self.target]}')

    def stop(self):
        self.velocities = [0.0] * 6

    def home(self):
        self.target = list(HOME_JOINTS)
        self.velocities = [0.0] * 6

    def set_joint_velocity(self, joint_idx: int, sign: float):
        # One joint at a time; reset others so a key press is unambiguous.
        self.velocities = [0.0] * 6
        self.velocities[joint_idx] = sign * self.speed

    def _tick(self):
        if self.target is None:
            return
        now = time.time()
        dt = now - self.last_tick_t
        self.last_tick_t = now

        if (now - self.last_key_t) > INACTIVITY_STOP_S:
            self.velocities = [0.0] * 6

        # Integrate target and clamp to URDF limits
        for i in range(6):
            self.target[i] += self.velocities[i] * dt
            lo, hi = JOINT_LIMITS[i]
            if self.target[i] < lo:
                self.target[i] = lo
            elif self.target[i] > hi:
                self.target[i] = hi

        # Publish a single-point trajectory with a short horizon — the
        # controller smoothly tracks the moving target as we keep ticking.
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = list(self.target)
        pt.time_from_start = Duration(
            sec=int(TRAJECTORY_HORIZON_S),
            nanosec=int((TRAJECTORY_HORIZON_S % 1) * 1e9))
        traj.points = [pt]
        self.pub.publish(traj)


def banner(node: 'ArmTeleop'):
    print()
    print('=' * 56)
    print('  ARM TELEOP — wrist-camera joint-space control')
    print('=' * 56)
    print('  a / d    PAN ± (joint1)')
    print('  w / s    TILT up / down (joint5)')
    print('  e / q    HIGHER / LOWER (joint2)')
    print('  r / f    EXTEND / RETRACT (joint3)')
    print('  z / x    ROLL ± (joint6)')
    print('  SPACE    stop (zero velocities)')
    print('  h        HOME (reset target)')
    print('  + / -    speed scale up / down')
    print('  CTRL-C   quit')
    print('-' * 56)
    print(f'  speed = {node.speed:.2f} rad/s, '
          f'auto-stop after {INACTIVITY_STOP_S:.2f}s idle')
    print('=' * 56)


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ArmTeleop()
    banner(node)

    # Wait briefly for /joint_states to seed the target.
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.target is not None:
            break
    if node.target is None:
        print('Warning: no /joint_states received yet; will seed on first message.')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(settings, timeout=0.05)
            if not key:
                continue
            if key == '\x03':  # CTRL-C
                break
            if key in KEY_BINDINGS:
                idx, sign = KEY_BINDINGS[key]
                node.set_joint_velocity(idx, sign)
                node.last_key_t = time.time()
                print(f'  [{key}]  joint{idx+1}{"+" if sign > 0 else "-"} '
                      f'@ {node.speed:.2f} rad/s')
            elif key == ' ':
                node.stop()
                print('  [STOP]')
            elif key in ('h', 'H'):
                node.home()
                node.last_key_t = time.time()
                print('  [HOME]')
            elif key in ('+', '='):
                node.speed = min(2.0, node.speed + SPEED_STEP)
                print(f'  speed up:  {node.speed:.2f} rad/s')
            elif key in ('-', '_'):
                node.speed = max(0.05, node.speed - SPEED_STEP)
                print(f'  speed dn:  {node.speed:.2f} rad/s')
            else:
                # Unknown key — safety stop
                node.stop()

    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print('\narm_teleop exited.')


if __name__ == '__main__':
    main()
