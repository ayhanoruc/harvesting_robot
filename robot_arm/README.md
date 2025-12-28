# Robot Arm - ROS2 Humble

4-DOF robotic arm simulation with Gazebo for ROS2 Humble.

Originally forked from [vedant-jad99/Robotic_Arm_Simulation_in_ROS](https://github.com/vedant-jad99/Robotic_Arm_Simulation_in_ROS), adapted for ROS2.

## Features

- **4 DOF arm**: hip (rotation), shoulder, elbow, wrist
- **2-finger gripper**: l_g_base, r_g_base
- **ros2_control integration**: JointTrajectoryController
- **Gazebo simulation**: Physics-based simulation with gazebo_ros2_control

## Robot Structure

```
world
└── base_link (fixed)
    └── torso (hip joint - continuous, Z-axis rotation)
        └── upper_arm (shoulder joint - revolute)
            └── lower_arm (elbow joint - revolute)
                └── hand (wrist joint - continuous)
                    ├── left_gripper_base (l_g_base - revolute)
                    │   └── left_gripper_finger (fixed)
                    └── right_gripper_base (r_g_base - revolute)
                        └── right_gripper_finger (fixed)
```

## Prerequisites

```bash
# ROS2 Humble
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros2-control
sudo apt install ros-humble-robot-state-publisher ros-humble-xacro
```

## Build

```bash
cd ~/harvesting_ws
colcon build --packages-select robot_arm
source install/setup.bash
```

## Usage

### Launch Gazebo Simulation

```bash
ros2 launch robot_arm bot.launch.py
```

### Control the Arm

In a new terminal:
```bash
source ~/harvesting_ws/install/setup.bash
ros2 run robot_arm write_pos.py
```

Commands:
- `move` - Set all 4 joint positions interactively
- `stop` - Return to home and exit
- `release` - Open gripper
- `close` - Close gripper
- `change` - Change single joint (format: `hip : 0.5`)

### Direct Topic Control

```bash
# Publish trajectory directly
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['hip', 'shoulder', 'elbow', 'wrist', 'l_g_base', 'r_g_base'], \
    points: [{positions: [0.5, 0.0, 0.5, 0.0, 0.0, 0.0], time_from_start: {sec: 1}}]}"
```

### Check Controllers

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## Joint Limits

| Joint | Type | Min | Max | Notes |
|-------|------|-----|-----|-------|
| hip | continuous | -π | π | Base rotation |
| shoulder | revolute | -1.57 | 0.70 | -90° to 40° |
| elbow | revolute | -1.57 | 1.57 | ±90° |
| wrist | continuous | -π | π | End rotation |
| l_g_base | revolute | 0 | 0.52 | Left gripper |
| r_g_base | revolute | -0.52 | 0 | Right gripper |

## Files

```
robot_arm/
├── package.xml              # ROS2 package manifest
├── CMakeLists.txt           # Build configuration
├── setup.py                 # Python package setup
├── launch/
│   └── bot.launch.py        # Main launch file
├── urdf/
│   └── mybot.urdf           # Robot description with ros2_control
├── yaml/
│   └── controllers.yaml     # Controller configuration
├── src/
│   └── write_pos.py         # Interactive control node
├── robot_arm/
│   └── __init__.py          # Python package
└── resource/
    └── robot_arm            # ament resource marker
```

## Troubleshooting

### Gazebo not starting
```bash
# Kill any existing Gazebo processes
killall gzserver gzclient
```

### Controllers not loading
```bash
# Check controller manager
ros2 service list | grep controller_manager
ros2 control list_controllers
```

### No joint states
```bash
ros2 topic echo /joint_states
```
