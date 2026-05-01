#!/usr/bin/env python3
"""
Simple WASD keyboard teleoperation for Husky.

Publishes geometry_msgs/Twist on /cmd_vel based on WASD keys.
Hold a key to move; release (any other key) to stop.

Keys:
    w        forward
    s        backward
    a        turn left  (rotate in place)
    d        turn right (rotate in place)
    q        forward + left
    e        forward + right
    z        backward + left
    c        backward + right
    space/x  emergency stop (zero velocity)
    +/-      increase/decrease speed scale
    CTRL-C   quit

Usage:
    ros2 run orchestrator wasd_teleop
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# Velocity scales (m/s, rad/s)
DEFAULT_LIN = 0.5
DEFAULT_ANG = 1.0
SPEED_STEP = 0.1

KEY_BINDINGS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    'q': (1.0, 1.0),
    'e': (1.0, -1.0),
    'z': (-1.0, 1.0),
    'c': (-1.0, -1.0),
    'x': (0.0, 0.0),
    ' ': (0.0, 0.0),
}


def get_key(settings, timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class WasdTeleop(Node):

    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lin_scale = DEFAULT_LIN
        self.ang_scale = DEFAULT_ANG
        self.cur_lin = 0.0
        self.cur_ang = 0.0
        # Continuous publish so the robot keeps moving while a key is held.
        # We treat each keypress as "set velocity until next press or stop".
        self.timer = self.create_timer(0.05, self._tick)

    def set_velocity(self, lin_dir, ang_dir):
        self.cur_lin = lin_dir * self.lin_scale
        self.cur_ang = ang_dir * self.ang_scale

    def stop(self):
        self.cur_lin = 0.0
        self.cur_ang = 0.0
        self._tick()

    def _tick(self):
        msg = Twist()
        msg.linear.x = self.cur_lin
        msg.angular.z = self.cur_ang
        self.pub.publish(msg)


def banner(node):
    print()
    print('=' * 50)
    print('  WASD Teleop  —  /cmd_vel')
    print('=' * 50)
    print('  w/s    forward / backward')
    print('  a/d    turn left / right')
    print('  q/e    forward+left / forward+right')
    print('  z/c    backward+left / backward+right')
    print('  x/SPC  stop')
    print('  +/-    speed scale up/down')
    print('  CTRL-C quit')
    print('-' * 50)
    print(f'  linear={node.lin_scale:.2f} m/s  '
          f'angular={node.ang_scale:.2f} rad/s')
    print('=' * 50)


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = WasdTeleop()
    banner(node)

    last_key = ''
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(settings, timeout=0.05)

            if not key:
                # No keypress — keep current velocity (held key behavior).
                continue

            if key == '\x03':  # CTRL-C
                break
            elif key in KEY_BINDINGS:
                lin, ang = KEY_BINDINGS[key]
                node.set_velocity(lin, ang)
                last_key = key
                print(f'  [{key}]  lin={node.cur_lin:+.2f}  ang={node.cur_ang:+.2f}')
            elif key in ('+', '='):
                node.lin_scale += SPEED_STEP
                node.ang_scale += SPEED_STEP
                print(f'  speed up:  lin={node.lin_scale:.2f}  ang={node.ang_scale:.2f}')
                # Re-apply current direction with new scale
                if last_key in KEY_BINDINGS:
                    lin, ang = KEY_BINDINGS[last_key]
                    node.set_velocity(lin, ang)
            elif key in ('-', '_'):
                node.lin_scale = max(0.05, node.lin_scale - SPEED_STEP)
                node.ang_scale = max(0.05, node.ang_scale - SPEED_STEP)
                print(f'  speed dn:  lin={node.lin_scale:.2f}  ang={node.ang_scale:.2f}')
                if last_key in KEY_BINDINGS:
                    lin, ang = KEY_BINDINGS[last_key]
                    node.set_velocity(lin, ang)
            else:
                # Unknown key — stop for safety
                node.stop()
                last_key = ''

    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print('\nWASD teleop exited.')


if __name__ == '__main__':
    main()
