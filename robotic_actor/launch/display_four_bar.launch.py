"""
Launch file to display the four-bar linkage with random motion in RViz.

This launch file:
1. Loads the four_bar.urdf
2. Starts robot_state_publisher to publish TF transforms
3. Starts robotic_actor_node to publish random joint states
4. Launches RViz for visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_robotic_actor = get_package_share_directory('robotic_actor')

    # Path to URDF file
    urdf_file = os.path.join(pkg_robotic_actor, 'urdf', 'four_bar.urdf')

    # Read URDF content
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }]
    )

    # Robotic Actor node (publishes random joint states)
    robotic_actor_node = Node(
        package='robotic_actor',
        executable='robotic_actor_node',
        name='robotic_actor_node',
        output='screen'
    )

    # Path to RViz config file
    rviz_config_file = os.path.join(pkg_robotic_actor, 'rviz', 'four_bar.rviz')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        robotic_actor_node,
        rviz_node
    ])
