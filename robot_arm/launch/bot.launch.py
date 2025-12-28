"""
ROS2 Launch file for 4-DOF Robot Arm in Gazebo

Launches:
  - Gazebo with empty world
  - robot_state_publisher (publishes URDF and TF)
  - Spawns robot in Gazebo (gazebo_ros2_control plugin handles hardware)
  - joint_state_broadcaster
  - arm_controller (joint trajectory controller)

Note: The gazebo_ros2_control plugin in the URDF creates the controller_manager
inside Gazebo, so we don't need a separate ros2_control_node.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Package paths
    pkg_path = get_package_share_directory('robot_arm')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Process xacro file
    xacro_file = os.path.join(pkg_path, 'urdf', 'mybot.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    robot_description = {'robot_description': robot_description_content}

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Spawn robot in Gazebo
    # The gazebo_ros2_control plugin in URDF handles the hardware interface
    # and creates the controller_manager
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot_arm',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Spawn joint_state_broadcaster (delayed to wait for Gazebo's controller_manager)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawn arm_controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay controller spawning until robot is spawned in Gazebo
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Delay arm_controller until joint_state_broadcaster is ready
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_joint_state_broadcaster,
        delay_arm_controller,
    ])
