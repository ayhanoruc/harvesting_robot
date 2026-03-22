cmake_minimum_required(VERSION 3.8)
project(robot_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/write_pos.py
  src/tcp_monitor.py
  robot_arm/landmark_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF, config, world files, and models
install(DIRECTORY
  urdf
  yaml
  worlds
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install other resources
install(DIRECTORY
  pictures
  DESTINATION share/${PROJECT_NAME}
)

# Environment hooks for Gazebo resource paths
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
