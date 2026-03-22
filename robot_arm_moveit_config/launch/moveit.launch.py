"""
MoveIt 2 Launch file for M1013 + Hand-E

Launches move_group and RViz for motion planning.
Connects to existing Gazebo simulation (run bot.launch.py first).

Usage:
    Terminal 1: ros2 launch robot_arm bot.launch.py
    Terminal 2: ros2 launch robot_arm_moveit_config moveit.launch.py
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    robot_arm_pkg = get_package_share_directory('robot_arm')
    moveit_config_pkg = get_package_share_directory('robot_arm_moveit_config')

    # Process URDF via xacro
    xacro_file = os.path.join(robot_arm_pkg, 'urdf', 'm1013_robocot.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': robot_description_content}

    # Load SRDF
    srdf_file = os.path.join(moveit_config_pkg, 'config', 'robot_arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # Load config files
    kinematics_yaml = load_yaml('robot_arm_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('robot_arm_moveit_config', 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml('robot_arm_moveit_config', 'config/ompl_planning.yaml')
    moveit_controllers_yaml = load_yaml('robot_arm_moveit_config', 'config/moveit_controllers.yaml')

    # MoveIt configuration
    moveit_config = {
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': joint_limits_yaml,
        'planning_pipelines': ['ompl'],
        'ompl': ompl_planning_yaml,
    }

    # Trajectory execution config
    trajectory_execution = {
        'moveit_manage_controllers': False,  # Gazebo manages controllers
        'trajectory_execution.allowed_execution_duration_scaling': 4.0,
        'trajectory_execution.allowed_goal_duration_margin': 2.0,
        'trajectory_execution.allowed_start_tolerance': 0.5,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # Planning scene config
    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Move Group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_config,
            moveit_controllers_yaml,
            trajectory_execution,
            planning_scene_monitor,
            {'use_sim_time': use_sim_time},
        ],
    )

    # RViz
    rviz_config_file = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Environment config file (from robot_arm package)
    env_config_file = os.path.join(robot_arm_pkg, 'config', 'environment_config.yaml')

    # Arm Commander node
    arm_commander_node = Node(
        package='robot_arm_moveit_config',
        executable='arm_commander.py',
        name='arm_commander',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'config_file': env_config_file},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        move_group_node,
        rviz_node,
        arm_commander_node,
    ])
