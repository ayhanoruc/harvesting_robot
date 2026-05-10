"""
Autonomous multi-cluster harvest demo launch.

Spawns Husky at the orchard SW corner (the original 'default' start used by
husky_test.launch.py) so harvest_orchestrator can autonomously drive through
multiple clusters in sequence — instead of being placed in front of one
cluster like husky_orchard_demo.launch.py does.

Default spawn: (x=3.0, y=5.0, yaw=0.0). Husky faces +X (along row 0),
trees at row 0 are at x=15.8, 17.0, 18.0 with y≈3.8. Aisle scout pose used
by harvest_orchestrator: (tree_x, ~4.85, yaw=0).

Internally just delegates to husky_orchard_demo.launch.py with overridden
spawn args — same Gazebo + bridges + controllers + corrected static TF.

Usage
-----
  Terminal A: ros2 launch robot_arm husky_autonomous.launch.py
  Terminal B: ros2 launch robot_arm_moveit_config moveit.launch.py
  Terminal C: ros2 run orchestrator simple_cluster_harvester
  Terminal D: ros2 run orchestrator harvest_orchestrator
              ros2 service call /harvest/start std_srvs/srv/Trigger '{}'
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('robot_arm')
    base_launch = os.path.join(pkg, 'launch', 'husky_orchard_demo.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'spawn_x': '3.0',
                'spawn_y': '5.0',
                'spawn_z': '0.0',
                'spawn_yaw': '0.0',
            }.items(),
        ),
    ])
