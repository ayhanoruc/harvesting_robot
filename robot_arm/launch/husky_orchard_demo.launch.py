"""
F1.7 Demo: Husky spawned in front of FIRST cluster (tree_000) for one-cluster
mock-pick run via simple_cluster_harvester.

Usage
-----
  Terminal A: ros2 launch robot_arm husky_orchard_demo.launch.py
  Terminal B: ros2 launch robot_arm_moveit_config moveit.launch.py
  Terminal C: ros2 run orchestrator simple_cluster_harvester
              ros2 service call /simple_harvest/start std_srvs/srv/Trigger '{}'

Spawn placement (tuned for tree_000 @ x=15.8, y=3.8, bolls on +Y aisle shell):
  Husky pose: x=15.8, y=4.7, yaw=-pi/2  (faces -Y → toward tree_000)
  → arm_mount lands at world y≈4.5; bolls at y∈[4.0,4.1] are 0.4-0.5m away.

Same world/bridges/controllers as husky_test.launch.py — only spawn pose changes
and a default world arg is exposed.
"""

import math
import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    AppendEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('robot_arm')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # World file is overridable via `world:=<filename>` launch arg.
    # Default: orchard_bolls.world (legacy sphere bolls).
    # Available: orchard.world (no bolls), orchard_bolls.world (spheres),
    #            orchard_pickable.world (cotton_pick_* models, no trees).
    world_arg = LaunchConfiguration('world')
    worlds_dir = os.path.join(pkg_path, 'worlds') + '/'

    controllers_yaml = os.path.join(pkg_path, 'yaml', 'controllers.yaml')
    bridge_params = os.path.join(pkg_path, 'config', 'husky_gz_bridge.yaml')

    xacro_file = os.path.join(pkg_path, 'urdf', 'husky_robocot.urdf.xacro')
    robot_description_content = xacro.process_file(
        xacro_file,
        mappings={
            'controllers_yaml': controllers_yaml,
            'standalone': 'false',
            'mobile': 'true',
        }
    ).toxml()

    # Cleanup (matches husky_test.launch.py)
    robot_description_content = re.sub(r'<\?xml.*?\?>', '', robot_description_content)
    robot_description_content = re.sub(r'<!--.*?-->', '', robot_description_content, flags=re.DOTALL)
    robot_description_content = re.sub(r'\s+', ' ', robot_description_content).strip()
    robot_description = {'robot_description': robot_description_content}

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Defaults target cotton_demo.world (compact template-instanced layout,
    # cluster_A_01 at world origin, Row 1 along +X, Husky aisle at Y=1.0).
    # For legacy orchard_bolls.world override: spawn_x:=15.8 spawn_y:=4.85.
    spawn_x = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y = LaunchConfiguration('spawn_y', default='1.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.0')
    # yaw=0 → Husky front along world +X (row line direction). Camera/arm
    # then look sideways (toward trees) via arm_commander's joint1=-π/2
    # startup rotation. Driving forward = advancing along the row.
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='0.0')

    # Resource paths
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_path, 'models'))
    set_ign_resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', os.path.join(pkg_path, 'models'))
    # Pickable cotton models are one level deeper (models/cotton_picks/cotton_pick_*).
    # Gazebo doesn't recurse, so the cotton_picks dir must also be on the path
    # for `model://cotton_pick_*` URIs to resolve.
    set_gz_picks_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_path, 'models', 'cotton_picks'))
    set_ign_picks_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', os.path.join(pkg_path, 'models', 'cotton_picks'))
    set_gz_mesh_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_path, '..'))
    set_ign_mesh_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', os.path.join(pkg_path, '..'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # Substitution-aware composition so `world:=...` arg takes effect
            'gz_args': ['-r -v4 ', worlds_dir, world_arg],
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

    # IMPORTANT: world->odom must equal Husky's Gazebo spawn pose so that
    # TF (world->odom->husky_base->arm->tcp) matches the actual world position.
    # DiffDrive plugin publishes odom->husky_base_link relative to spawn, so
    # offsetting world->odom by spawn pose closes the loop.
    static_tf_world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_odom',
        arguments=[
            '--x', spawn_x,
            '--y', spawn_y,
            '--z', spawn_z,
            '--yaw', spawn_yaw,
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'odom',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'husky_robocot',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
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
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen',
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen',
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen',
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=8.0, actions=[joint_state_broadcaster_spawner])],
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
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='cotton_demo.world',
                              description='World file under robot_arm/worlds/. '
                                          'Default: cotton_demo.world (compact template-instanced '
                                          '12-cluster field). Other: orchard.world, orchard_bolls.world, '
                                          'orchard_pickable.world. '
                                          'For legacy orchard_bolls.world override: '
                                          'spawn_x:=15.8 spawn_y:=4.85.'),
        DeclareLaunchArgument('spawn_x', default_value='0.0',
                              description='Husky spawn X (default: 0.0 = in front of cluster_A_01 '
                                          'in cotton_demo.world; use 15.8 for orchard_bolls.world)'),
        DeclareLaunchArgument('spawn_y', default_value='1.0',
                              description='Husky spawn Y (default: 1.0 = +Y aisle side of Row 1 '
                                          'in cotton_demo.world, ~0.65m from nearest boll; '
                                          'use 4.85 for orchard_bolls.world)'),
        DeclareLaunchArgument('spawn_z', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0',
                              description='Husky yaw (default: 0 → front along +X = row line; '
                                          'arm rotates right via joint1=-π/2 to look at trees)'),
        set_gz_resource_path,
        set_ign_resource_path,
        set_gz_picks_path,
        set_ign_picks_path,
        set_gz_mesh_path,
        set_ign_mesh_path,
        gazebo,
        robot_state_publisher,
        static_tf_world_to_odom,
        spawn_entity,
        ros_gz_bridge,
        ros_gz_image_bridge,
        delay_joint_state_broadcaster,
        delay_arm_controller,
        delay_gripper_controller,
    ])
