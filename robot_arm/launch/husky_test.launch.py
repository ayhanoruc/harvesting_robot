"""
Sanity Test #2: Husky + M1013 + Reservoir in orchard.world

Spawns the full Husky composition (Husky body + 4 wheels + mount + arm + reservoir)
in the orchard world. F1.4 sanity check — verifies URDF composition is valid and
all parts render correctly. Robot is statically fixed to world (F2 will free it).

Usage:
    ros2 launch robot_arm husky_test.launch.py
"""

import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    AppendEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('robot_arm')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_path, 'worlds', 'orchard.world')
    controllers_yaml = os.path.join(pkg_path, 'yaml', 'controllers.yaml')
    bridge_params = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')

    # Process the Husky composition xacro
    xacro_file = os.path.join(pkg_path, 'urdf', 'husky_robocot.urdf.xacro')
    robot_description_content = xacro.process_file(
        xacro_file,
        mappings={
            'controllers_yaml': controllers_yaml,
            'standalone': 'false',
        }
    ).toxml()

    # URDF cleanup (same as bot.launch.py — fixes parser issues)
    robot_description_content = re.sub(r'<\?xml.*?\?>', '', robot_description_content)
    robot_description_content = re.sub(r'<!--.*?-->', '', robot_description_content, flags=re.DOTALL)
    robot_description_content = re.sub(r'\s+', ' ', robot_description_content).strip()

    robot_description = {'robot_description': robot_description_content}

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Resource paths
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_path, 'models')
    )
    set_ign_resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.path.join(pkg_path, 'models')
    )
    set_gz_mesh_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_path, '..')
    )
    set_ign_mesh_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.path.join(pkg_path, '..')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v4 {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Spawn at orchard SW corner facing tree row 0
    # Tree bbox X[4,41] Y[3,35]; place husky at (3, 5) with yaw 0 (facing +X = toward trees)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'husky_robocot',
            '-x', '3.0',
            '-y', '5.0',
            '-z', '0.0',
        ],
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image', '/camera/depth_image'],
        output='screen',
        remappings=[
            ('/camera/image', '/camera/color/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        set_gz_resource_path,
        set_ign_resource_path,
        set_gz_mesh_path,
        set_ign_mesh_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
        ros_gz_image_bridge,
        delay_joint_state_broadcaster,
        delay_arm_controller,
        delay_gripper_controller,
    ])
