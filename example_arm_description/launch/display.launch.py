"""
Launch file for visualizing the example arm in RViz.

Launches:
  - robot_state_publisher: publishes /robot_description and TF transforms
  - joint_state_publisher_gui: GUI sliders to control joint positions
  - rviz2: visualization with pre-configured display settings
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_name = 'example_arm_description'
    pkg_share = get_package_share_directory(pkg_name)

    # Path to xacro file
    xacro_file = os.path.join(pkg_share, 'description', 'example_robot.urdf.xacro')

    # Process xacro -> URDF XML string
    robot_description_content = xacro.process_file(xacro_file).toxml()

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        # Launch argument for simulation time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # robot_state_publisher node
        # - Publishes /robot_description topic (URDF as string)
        # - Publishes TF transforms based on joint_states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time
            }]
        ),

        # joint_state_publisher_gui node
        # - Provides GUI sliders to manually set joint positions
        # - Publishes /joint_states topic
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # RViz2 node
        # - 3D visualization
        # - Uses saved config from rviz/ folder
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
