"""
MoveIt 2 Demo Launch File with ros2_control

Launches:
  - robot_state_publisher (publishes URDF and TF)
  - ros2_control_node (hardware interface)
  - joint_state_broadcaster (publishes joint states)
  - arm_controller (joint trajectory controller)
  - move_group (MoveIt's main node)
  - rviz2 with MoveIt plugin
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def load_yaml(package_name, file_path):
    """Load a yaml file from a package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():

    # Package paths
    arm_description_pkg = get_package_share_directory('example_arm_description')
    moveit_config_pkg = get_package_share_directory('example_arm_moveit_config')

    # Process URDF from xacro
    xacro_file = os.path.join(arm_description_pkg, 'description', 'example_robot.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': robot_description_content}

    # Load SRDF
    srdf_file = os.path.join(moveit_config_pkg, 'config', 'example_arm.srdf')
    with open(srdf_file, 'r') as file:
        robot_description_semantic = {'robot_description_semantic': file.read()}

    # Load config files
    kinematics_yaml = load_yaml('example_arm_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('example_arm_moveit_config', 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml('example_arm_moveit_config', 'config/ompl_planning.yaml')
    moveit_controllers_yaml = load_yaml('example_arm_moveit_config', 'config/moveit_controllers.yaml')

    # ros2_controllers config path
    ros2_controllers_yaml = os.path.join(moveit_config_pkg, 'config', 'ros2_controllers.yaml')

    # Trajectory execution enabled
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'allow_trajectory_execution': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # Planning scene monitor parameters
    planning_scene_monitor_params = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Launch argument for sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ===== NODES =====

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # ros2_control_node - manages hardware interface
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_yaml],
        output='screen',
    )

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawn arm_controller (after joint_state_broadcaster)
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Planning pipeline configuration - use OMPL with time parameterization
    planning_pipeline_config = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # MoveIt Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'robot_description_planning': joint_limits_yaml},
            planning_pipeline_config,
            moveit_controllers_yaml,
            trajectory_execution,
            planning_scene_monitor_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    # RViz with MoveIt plugin
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'use_sim_time': use_sim_time},
        ],
    )

    # Event handler: spawn arm_controller after joint_state_broadcaster is ready
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller,
        move_group_node,
        rviz_node,
    ])
