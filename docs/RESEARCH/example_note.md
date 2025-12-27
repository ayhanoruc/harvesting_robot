CAUTION: THE CODE MIGHT BE OLD to work with ros2 or gazebo
How To Run
Build the package with colcon.
Launch the robot_state_publisher launch file with ros2 launch urdf_example rsp.launch.py.
Launch joint_state_publisher_gui with ros2 run joint_state_publisher_gui joint_state_publisher_gui. You may need to install it if you don't have it already.
Launch RViz with rviz2
To replicate the RViz display shown in the video you will want to

Set your fixed frame to world
Add a RobotModel display, with the topic set to /robot_description, and alpha set to 0.8
Add a TF display with names enabled.


----
  Step 1: Build the package
  cd ~/harvesting_ws
  colcon build --packages-select example_arm_description

  Step 2: Source the workspace
  source install/setup.bash

  Step 3: Launch RViz with the arm
  ros2 launch example_arm_description display.launch.py