#!/bin/bash
# RoboCot Docker entrypoint
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
exec "$@"
