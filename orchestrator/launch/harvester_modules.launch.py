"""
Harvester Modules — bundle launch for the 5 nodes you keep running together.

Launches in a sensible startup order, all in the same terminal so logs are
interleaved with a per-node prefix (e.g. [cv_boll_detector-1]).

Bundled nodes:
  1. cv_boll_detector              — /yolo/detect (classical CV)
  2. depth_processor               — /depth_processor/pixel_to_3d
  3. cluster_scanner               — /cluster_scan/run (sweep + 3D + bbox)
  4. simple_cluster_harvester      — /simple_harvest/start (pick subroutine)
  5. cluster_harvester             — /cluster_harvester/run (scan + pick loop)

NOT bundled (run separately):
  - husky_orchard_demo.launch.py  (Gazebo + Husky URDF)
  - moveit.launch.py              (move_group + arm_commander + gripper)
  - row_navigator                  (top-level row pipeline — your trigger)

Usage:
  ros2 launch orchestrator harvester_modules.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Small startup delay between nodes so service availability lines up
    # nicely in the log; ROS2 actions/services already handle reconnect.
    return LaunchDescription([
        Node(
            package='orchestrator',
            executable='cv_boll_detector',
            name='cv_boll_detector',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='orchestrator',
            executable='depth_processor',
            name='depth_processor',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='orchestrator',
            executable='cluster_scanner',
            name='cluster_scanner',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='orchestrator',
            executable='simple_cluster_harvester',
            name='simple_cluster_harvester',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='orchestrator',
            executable='cluster_harvester',
            name='cluster_harvester',
            output='screen',
            emulate_tty=True,
        ),
    ])
