wsl -d Ubuntu-22.04

colcon build --packages-select robot_arm robot_arm_moveit_config

source install/setup.bash

export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch robot_arm bot.launch.py

ros2 launch robot_arm_moveit_config moveit.launch.py

ros2 run rqt_image_view rqt_image_view

ros2 param set /arm_commander target_name cluster_1

ros2 service call /go_to_named std_srvs/srv/SetBool "{data: true}"

colcon build --packages-select robot_arm robot_arm_moveit_config orchestrator

ros2 run orchestrator explorer

 ros2 service call /explorer/start_scan std_srvs/srv/Trigger "{}"