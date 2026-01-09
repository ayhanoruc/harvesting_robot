"""
Simple launch file to visualize Braccio arm with cotton_field world in Gazebo.
For screenshot/visual purposes only - not functional.

Usage:
  cd /mnt/c/Users/ayhan/harvesting_ws
  source install/setup.bash
  export LIBGL_ALWAYS_SOFTWARE=1
  ros2 launch /mnt/c/Users/ayhan/harvesting_ws/src/docs/figures/braccio_visual.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Paths
    pkg_robot_arm = get_package_share_directory('robot_arm')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # World file from robot_arm package
    world_file = os.path.join(pkg_robot_arm, 'worlds', 'cotton_field.world')

    # URDF file (absolute path)
    urdf_file = '/mnt/c/Users/ayhan/harvesting_ws/src/docs/figures/braccio_visual.urdf'

    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    robot_description = {'robot_description': robot_description_content}

    # Set resource path for cotton_cluster model
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_robot_arm, 'models')
    )
    set_ign_resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.path.join(pkg_robot_arm, 'models')
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v4 {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'braccio_arm',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
