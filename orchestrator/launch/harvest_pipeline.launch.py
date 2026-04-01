"""
Harvest Pipeline Launch — All vision + orchestration nodes

Run AFTER bot.launch.py (Gazebo) and moveit.launch.py (MoveIt + arm_commander).

Launches:
  - explorer (simplified panoramic scan: 3 positions)
  - real_yolo_detector (YOLO inference)
  - depth_processor (pixel -> 3D)
  - camera_focus (pixel error -> joint adjustment)
  - spatial_detection_pipeline (YOLO + focus + depth coordination)
  - harvest_executor (single-boll pick-and-place)
  - orchestrator_node (main state machine)

Usage:
    Terminal 1: ros2 launch robot_arm bot.launch.py
    Terminal 2: ros2 launch robot_arm_moveit_config moveit.launch.py
    Terminal 3: ros2 launch orchestrator harvest_pipeline.launch.py
    Then:       ros2 service call /orchestrator/start_harvest std_srvs/srv/Trigger "{}"
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package paths
    robot_arm_pkg = get_package_share_directory('robot_arm')

    # Common parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    env_config_file = os.path.join(robot_arm_pkg, 'config', 'environment_config.yaml')

    # ── Explorer (simplified panoramic scan) ────────────────
    # 3 pan positions from HOME, j1 only, no arm movement
    explorer_node = Node(
        package='orchestrator',
        executable='explorer',
        name='explorer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': env_config_file,
            # Simplified scan: 3 columns (pan), 1 row (HOME tilt)
            'pan_joint1_angles': [-0.50, 0.0, 0.50],
            'pan_joint2_range': [-0.922],
            'pan_joint3_range': [2.4494],
            'pan_joint4_range': [0.0],
            'pan_joint5_range': [-1.3000],
            'pan_joint6_range': [0.0],
            'pan_pause_duration': 2.0,
            'pan_move_duration': 1.5,
            'enable_detection': True,
        }],
    )

    # ── YOLO Detector ───────────────────────────────────────
    real_yolo_detector_node = Node(
        package='orchestrator',
        executable='real_yolo_detector',
        name='real_yolo_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'confidence': 0.5,
            'camera_topic': '/camera/color/image_raw',
            'cluster_pixel_distance': 150,
            'save_images': True,
        }],
    )

    # ── Depth Processor ─────────────────────────────────────
    depth_processor_node = Node(
        package='orchestrator',
        executable='depth_processor',
        name='depth_processor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_info_topic': '/camera/depth/camera_info',
            'depth_image_topic': '/camera/depth/image_raw',
            'camera_frame': 'camera_optical_frame',
            'world_frame': 'world',
        }],
    )

    # ── Camera Focus ────────────────────────────────────────
    camera_focus_node = Node(
        package='orchestrator',
        executable='camera_focus',
        name='camera_focus',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Spatial Detection Pipeline ──────────────────────────
    spatial_detection_node = Node(
        package='orchestrator',
        executable='spatial_detection_pipeline',
        name='spatial_detection_pipeline',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': env_config_file,
            'focus_iterations': 0,   # skip focus in sim (too slow)
            'save_images': True,
        }],
    )

    # ── Harvest Executor ────────────────────────────────────
    harvest_executor_node = Node(
        package='orchestrator',
        executable='harvest_executor',
        name='harvest_executor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': env_config_file,
            'pre_grasp_offset': 0.15,
            'lift_height': 0.15,
        }],
    )

    # ── Orchestrator (Main State Machine) ───────────────────
    orchestrator_node = Node(
        package='orchestrator',
        executable='orchestrator_node',
        name='orchestrator_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': env_config_file,
            'pre_grasp_offset': 0.15,
            'scan_timeout': 600.0,
            'camera_settle_time': 3.0,
            'use_vision_for_scan': True,
            'use_vision_for_bolls': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time'),
        explorer_node,
        real_yolo_detector_node,
        depth_processor_node,
        camera_focus_node,
        spatial_detection_node,
        harvest_executor_node,
        orchestrator_node,
    ])
