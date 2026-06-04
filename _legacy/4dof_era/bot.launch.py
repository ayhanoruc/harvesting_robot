"""
ROS2 Launch file for M1013 + Hand-E Robot in Gazebo Sim (Ignition Fortress)

Launches:
  - Gazebo Sim with cotton_field world
  - robot_state_publisher (publishes URDF and TF)
  - Spawns robot in Gazebo (gz_ros2_control plugin handles hardware)
  - ROS-Gazebo bridges for topics
  - joint_state_broadcaster
  - arm_controller (6-DOF joint trajectory controller)
  - gripper_controller (Hand-E prismatic finger)
  - landmark_publisher (static TF + collision objects)

Note: The gz_ros2_control plugin in the URDF creates the controller_manager
inside Gazebo, so we don't need a separate ros2_control_node.
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
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Package paths
    pkg_path = get_package_share_directory('robot_arm')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # World file
    world_file = os.path.join(pkg_path, 'worlds', 'cotton_field.world')

    # Controllers yaml file (injected into xacro)
    controllers_yaml = os.path.join(pkg_path, 'yaml', 'controllers.yaml')

    # Bridge config
    bridge_params = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')

    # Process xacro file with mappings
    xacro_file = os.path.join(pkg_path, 'urdf', 'm1013_robocot.urdf.xacro')
    robot_description_content = xacro.process_file(
        xacro_file,
        mappings={'controllers_yaml': controllers_yaml}
    ).toxml()

    # CRITICAL FIX: Clean up URDF to prevent parser errors
    # Remove XML declaration (causes parser confusion with '?' and '=')
    robot_description_content = re.sub(r'<\?xml.*?\?>', '', robot_description_content)
    # Remove comments (reduces size significantly)
    robot_description_content = re.sub(r'<!--.*?-->', '', robot_description_content, flags=re.DOTALL)
    # Collapse whitespace (further reduces size)
    robot_description_content = re.sub(r'\s+', ' ', robot_description_content).strip()

    robot_description = {'robot_description': robot_description_content}

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Set GZ_SIM_RESOURCE_PATH for model discovery
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_path, 'models')
    )

    # Also set IGN_GAZEBO_RESOURCE_PATH for Fortress compatibility
    set_ign_resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.path.join(pkg_path, 'models')
    )

    # Add share/ parent so Gazebo can resolve package://robot_arm/meshes/... URIs
    set_gz_mesh_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_path, '..')
    )
    set_ign_mesh_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.path.join(pkg_path, '..')
    )

    # Gazebo Sim launch (Ignition Fortress)
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
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Spawn robot in Gazebo Sim using ros_gz_sim/create
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robot_arm',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # ROS-Gazebo bridge for topics
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    # Image bridge (more efficient for camera images)
    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image', '/camera/depth_image'],
        output='screen',
        # Remap gz topics to ROS topics
        remappings=[
            ('/camera/image', '/camera/color/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
        ]
    )

    # Spawn joint_state_broadcaster (delayed to wait for controller_manager)
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

    # Spawn gripper_controller
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
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

    # Delay gripper_controller until arm_controller is ready
    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # Environment config file
    config_file = os.path.join(pkg_path, 'config', 'environment_config.yaml')

    # Landmark publisher (TF frames + collision objects)
    landmark_publisher = Node(
        package='robot_arm',
        executable='landmark_publisher.py',
        name='landmark_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'config_file': config_file}
        ]
    )

    # Delay landmark publisher until gripper_controller is ready
    delay_landmark_publisher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[landmark_publisher],
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
        delay_landmark_publisher,
    ])
