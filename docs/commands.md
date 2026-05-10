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

ros2 service call /explorer/panoramic_scan std_srvs/srv/Trigger "{}"


ros2 run orchestrator wasd_teleop


# Build with detection pipeline
colcon build --packages-select harvester_interfaces orchestrator robot_arm robot_arm_moveit_config


# Run panoramic scan with detection (enable_detection=true by default)
ros2 service call /explorer/panoramic_scan std_srvs/srv/Trigger "{}"

# Disable detection for faster scanning
ros2 param set /explorer enable_detection false
ros2 service call /explorer/panoramic_scan std_srvs/srv/Trigger "{}"

# Manual detection commands
ros2 service call /yolo/detect harvester_interfaces/srv/YoloDetect "{}"
ros2 service call /detection/run_at_position std_srvs/srv/Trigger "{}"
ros2 service call /detection/validate std_srvs/srv/Trigger "{}"
ros2 service call /detection/print_results std_srvs/srv/Trigger "{}"
ros2 service call /detection/clear std_srvs/srv/Trigger "{}"

 
  ros2 run orchestrator camera_focus &
  ros2 run orchestrator real_yolo_detector &
  ros2 run orchestrator depth_processor &
  ros2 run orchestrator spatial_detection_pipeline


  # Terminal 2: Build & run
  cd /mnt/c/Users/ayhan/harvesting_ws
  colcon build --packages-select orchestrator
  source install/setup.bash

  # Run YOLO detector
  ros2 run orchestrator real_yolo_detector


  mkdir -p /mnt/c/Users/ayhan/harvesting_ws/yolo_output && cp /tmp/yolo_detections/* /mnt/c/Users/ayhan/harvesting_ws/yolo_output/


  ros2 service call /detection/run_at_position std_srvs/srv/Trigger "{}"


   ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller


---

ros2 launch robot_arm husky_autonomous.launch.py

ros2 launch robot_arm_moveit_config moveit.launch.py

ros2 run orchestrator simple_cluster_harvester

ros2 run orchestrator harvest_orchestrator

ros2 service call /harvest/start std_srvs/srv/Trigger '{}'