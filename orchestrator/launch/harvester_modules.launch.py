"""
Harvester Modules — bundle launch for the 5 nodes you keep running together.

Launches in a sensible startup order, all in the same terminal so logs are
interleaved with a per-node prefix (e.g. [real_yolo_detector-1]).

Bundled nodes:
  1. real_yolo_detector            — /yolo/detect + /yolo/detect_clusters (YOLO11n)
  2. depth_processor               — /depth_processor/pixel_to_3d
  3. cluster_scanner               — /cluster_scan/run (sweep + 3D + bbox)
  4. simple_cluster_harvester      — /simple_harvest/start (pick subroutine)
  5. cluster_harvester             — /cluster_harvester/run (scan + pick loop)

NOT bundled (run separately):
  - husky_orchard_demo.launch.py  (Gazebo + Husky URDF)
  - moveit.launch.py              (move_group + arm_commander + gripper)
  - row_navigator                  (top-level row pipeline — your trigger)

Switched from cv_boll_detector to real_yolo_detector (2026-05-31). Service
interface identical: /yolo/detect + /yolo/detect_clusters with bbox+label,
so downstream (cluster_scanner, simple_cluster_harvester, cluster_harvester,
spatial_detection_pipeline) need no changes. Model auto-loaded from
orchestrator/models/best.pt (yolo11n trained on Roboflow cotton-boll-and-
cluster v5 — classes: cotton_boll, unripe-cotton).

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
            executable='real_yolo_detector',
            name='real_yolo_detector',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'confidence':              0.30,
                'camera_topic':            '/camera/color/image_raw',
                'cluster_pixel_distance':  150,
                'save_images':             True,
            }],
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
