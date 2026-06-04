# Project Checkpoint - ROS2 Arm Control Setup

**Date:** 2025-12-28
**Status:** Joint-space arm commander ready for testing

---

## Overview

This is a ROS2 Humble robotic cotton harvesting system. The main goal was to move from mock implementations to real arm control with MoveIt 2.

## What Was Built

### 1. `example_arm_description` Package
Robot description package with URDF and visualization.

**Key Files:**
- `description/example_robot.urdf.xacro` - Main robot URDF with:
  - `slider_joint` (prismatic, 0-2m)
  - `arm_joint` (revolute, 0-90°)
  - TCP frame chain: `camera_link → tool0 → tcp` (red sphere visual)
- `description/ros2_control.xacro` - Mock hardware interface using `mock_components/GenericSystem`
- `launch/display.launch.py` - RViz visualization
- `example_arm_description/tcp_monitor.py` - Node that reads TCP position from TF (FK demo)

**TCP Position:** At joints `[0, 0]`, TCP is at approximately `(1.61, 1.0, 0.43)` in world frame.

### 2. `example_arm_moveit_config` Package
MoveIt 2 configuration with ros2_control integration.

**Config Files:**
- `config/example_arm.srdf` - Planning group "arm" with both joints, tcp as end-effector
- `config/kinematics.yaml` - KDL solver config
- `config/joint_limits.yaml` - Velocity/acceleration limits
- `config/ompl_planning.yaml` - OMPL planner configurations
- `config/ros2_controllers.yaml` - ros2_control controller config:
  - `joint_state_broadcaster` - Publishes joint states
  - `arm_controller` - JointTrajectoryController for execution
- `config/moveit_controllers.yaml` - MoveIt→ros2_control bridge config
- `config/moveit.rviz` - RViz config with MotionPlanning plugin
- `config/target_positions.yaml` - Named joint-space targets

**Launch:**
- `launch/demo.launch.py` - Launches full MoveIt stack:
  - robot_state_publisher
  - ros2_control_node (mock hardware)
  - joint_state_broadcaster
  - arm_controller
  - move_group (OMPL planner)
  - rviz2

**Arm Commander Node:**
- `example_arm_moveit_config/arm_commander.py` - Python node for autonomous movement
  - Uses joint-space goals (NOT Cartesian - see limitations)
  - Loads targets from YAML
  - Service interface: `/go_to_target` (std_srvs/SetBool)
  - Parameter: `target` (string, e.g., "HOME", "EXTENDED_LOW")

---

## Available Targets

From `target_positions.yaml`:

| Target | Joints [slider, arm] | Description |
|--------|---------------------|-------------|
| HOME | [0.0, 0.0] | Home position |
| EXTENDED_LOW | [1.5, 0.0] | Slider extended, arm horizontal |
| RETRACTED_MID | [0.0, 0.785] | Slider home, arm at 45° |
| MID_HIGH | [1.0, 1.57] | Slider mid, arm vertical |
| FULL_REACH | [2.0, 0.3] | Maximum slider, slight arm angle |
| SCAN_LEFT | [0.0, 0.5] | Left scan position |
| SCAN_RIGHT | [2.0, 0.5] | Right scan position |

---

## How to Run

### Prerequisites
```bash
# Required packages (already installed)
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
```

### Build
```bash
cd ~/harvesting_ws
colcon build --packages-select example_arm_description example_arm_moveit_config
source install/setup.bash
```

### Launch MoveIt Demo
```bash
ros2 launch example_arm_moveit_config demo.launch.py
```

This opens RViz with MotionPlanning plugin. You can:
- Drag the interactive marker to set goals
- Click "Plan & Execute" to move the arm

### Run Arm Commander (Autonomous Movement)
In a new terminal:
```bash
source ~/harvesting_ws/install/setup.bash
ros2 run example_arm_moveit_config arm_commander
```

To move to a target:
```bash
ros2 param set /arm_commander target EXTENDED_LOW
ros2 service call /go_to_target std_srvs/srv/SetBool "{data: true}"
```

### Monitor TCP Position
```bash
ros2 run example_arm_description tcp_monitor
```

---

## Architecture

```
                    ┌─────────────────┐
                    │   arm_commander │
                    │  (Python node)  │
                    └────────┬────────┘
                             │ MoveGroup.action
                             ▼
┌──────────────────────────────────────────────────────────┐
│                      move_group                          │
│  - OMPL motion planning                                  │
│  - Trajectory time parameterization                      │
│  - Collision checking                                    │
└────────────────────────┬─────────────────────────────────┘
                         │ FollowJointTrajectory.action
                         ▼
┌──────────────────────────────────────────────────────────┐
│                    arm_controller                        │
│  (joint_trajectory_controller)                           │
└────────────────────────┬─────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────────┐
│                  ros2_control_node                       │
│  mock_components/GenericSystem (simulated hardware)      │
└──────────────────────────────────────────────────────────┘
```

---

## Known Limitations

### 2-DOF Arm Kinematics
- The arm has only 2 joints: slider (X translation) and arm_joint (rotation in XZ plane)
- **Y position is FIXED at 1.0** - cannot move laterally
- TCP moves only in the XZ plane
- **Cartesian IK often fails** because arbitrary XYZ positions may be unreachable

### Why Joint-Space Goals
We switched from Cartesian goals to joint-space goals because:
1. Cartesian IK solver frequently returned "Unable to sample any valid states"
2. Position-only constraints (ignoring orientation) still failed
3. For a 2-DOF arm, joint-space goals are 100% reliable

### WSL2 Graphics
If RViz displays incorrectly (gray/clipped), use:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

---

## Problems Solved

| Problem | Solution |
|---------|----------|
| MoveIt not installed | `sudo apt install ros-humble-moveit` |
| ros2_control not installed | `sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers` |
| Wrong robot in RViz | Fixed RViz config to use Topic for robot_description |
| Trajectory timestamp errors | Switched to OMPL with AddTimeOptimalParameterization |
| Fake controller manager failures | Implemented proper ros2_control with mock hardware |
| Cartesian IK failures | Switched to joint-space goals |

---

## Current Status

- [x] URDF with TCP frame
- [x] ros2_control mock hardware
- [x] MoveIt 2 motion planning
- [x] RViz Plan & Execute works
- [x] arm_commander node with joint-space goals
- [ ] **PENDING:** Test arm_commander movement to targets

---

## Next Steps

1. **Test arm_commander:** Verify it successfully moves between named targets
2. **Add more targets:** Define application-specific positions as needed
3. **Integrate with orchestrator:** Connect arm_commander to main harvesting workflow
4. **Real hardware:** Replace mock_components with actual hardware interface when available

---

## File Structure

```
harvesting_ws/
├── src/
│   ├── example_arm_description/
│   │   ├── description/
│   │   │   ├── example_robot.urdf.xacro
│   │   │   └── ros2_control.xacro
│   │   ├── launch/
│   │   │   └── display.launch.py
│   │   ├── example_arm_description/
│   │   │   └── tcp_monitor.py
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   └── example_arm_moveit_config/
│       ├── config/
│       │   ├── example_arm.srdf
│       │   ├── kinematics.yaml
│       │   ├── joint_limits.yaml
│       │   ├── ompl_planning.yaml
│       │   ├── ros2_controllers.yaml
│       │   ├── moveit_controllers.yaml
│       │   ├── moveit.rviz
│       │   └── target_positions.yaml
│       ├── launch/
│       │   └── demo.launch.py
│       ├── example_arm_moveit_config/
│       │   └── arm_commander.py
│       ├── package.xml
│       └── setup.py
│
└── docs/
    ├── CHECKPOINT.md (this file)
    ├── NOTES.md
    └── RESEARCH/
        └── example_note.md
```

---

## Quick Reference Commands

```bash
# Build
colcon build --packages-select example_arm_description example_arm_moveit_config

# Launch MoveIt
ros2 launch example_arm_moveit_config demo.launch.py

# Run arm commander
ros2 run example_arm_moveit_config arm_commander

# Move to target
ros2 param set /arm_commander target HOME
ros2 service call /go_to_target std_srvs/srv/SetBool "{data: true}"

# Monitor TCP
ros2 run example_arm_description tcp_monitor

# Check controllers
ros2 control list_controllers
```

---

## 3. `robot_arm` Package (iris_arm - 4-DOF Gazebo Simulation)

Downloaded from [github.com/aieask/iris_arm](https://github.com/aieask/iris_arm) and converted from ROS1 to ROS2 Humble.

### Robot Structure

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

### ROS1 to ROS2 Conversion

| File | Changes |
|------|---------|
| `package.xml` | catkin format 2 → ament format 3 |
| `CMakeLists.txt` | catkin → ament_cmake + ament_python |
| `setup.py` | NEW - Python package setup |
| `launch/bot.launch.py` | XML → Python launch file |
| `src/write_pos.py` | rospy → rclpy |
| `urdf/mybot.urdf.xacro` | ROS1 transmissions → ros2_control, gazebo_ros → gazebo_ros2_control |
| `yaml/controllers.yaml` | ROS1 format → ROS2 controller_manager format |

### Key Files

- `urdf/mybot.urdf.xacro` - Robot description with ros2_control hardware interface
- `yaml/controllers.yaml` - Controller configuration for joint_trajectory_controller
- `launch/bot.launch.py` - Launches Gazebo + robot + controllers
- `src/write_pos.py` - Interactive CLI for arm control

### Joint Limits

| Joint | Type | Min | Max | Description |
|-------|------|-----|-----|-------------|
| hip | continuous | -π | π | Base rotation (Z-axis) |
| shoulder | revolute | -1.57 | 0.70 | Upper arm pitch |
| elbow | revolute | -1.57 | 1.57 | Lower arm pitch |
| wrist | continuous | -π | π | Hand rotation |
| l_g_base | revolute | 0 | 0.52 | Left gripper |
| r_g_base | revolute | -0.52 | 0 | Right gripper |

### How to Run

```bash
# Prerequisites
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros2-control

# Build
cd ~/harvesting_ws
colcon build --packages-select robot_arm
source install/setup.bash

# Launch Gazebo simulation
ros2 launch robot_arm bot.launch.py
```

### Control Commands

**Move arm via topic:**
```bash
# Move to a pose (hip, shoulder, elbow, wrist, l_gripper, r_gripper)
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['hip', 'shoulder', 'elbow', 'wrist', 'l_g_base', 'r_g_base'], \
    points: [{positions: [1.0, -0.5, 0.8, 0.3, 0.0, 0.0], time_from_start: {sec: 2}}]}"

# Return to home
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['hip', 'shoulder', 'elbow', 'wrist', 'l_g_base', 'r_g_base'], \
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"

# Close gripper
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['hip', 'shoulder', 'elbow', 'wrist', 'l_g_base', 'r_g_base'], \
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.5, -0.5], time_from_start: {sec: 1}}]}"
```

**Interactive CLI:**
```bash
ros2 run robot_arm write_pos.py
# Commands: move, stop, release, close, change
```

**Check status:**
```bash
ros2 topic echo /joint_states --once
ros2 control list_controllers
```

### Architecture (Gazebo Simulation)

```
┌─────────────────────────────────────────────────────────────┐
│                         Gazebo                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │           gazebo_ros2_control plugin                │   │
│  │  - Loads ros2_control hardware interface            │   │
│  │  - Creates controller_manager inside Gazebo         │   │
│  │  - Simulates joint physics                          │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                              │
          ┌───────────────────┴───────────────────┐
          ▼                                       ▼
┌─────────────────────┐               ┌─────────────────────┐
│ joint_state_        │               │   arm_controller    │
│ broadcaster         │               │ (JointTrajectory    │
│ → /joint_states     │               │  Controller)        │
└─────────────────────┘               └─────────────────────┘
                                                │
                                                ▼
                                      /arm_controller/
                                      joint_trajectory
```

### Problems Solved

| Problem | Solution |
|---------|----------|
| `gazebo_ros2_control/GazeboSystem` not found | `sudo apt install ros-humble-gazebo-ros2-control` |
| Standalone ros2_control_node crashes | Remove it - gazebo_ros2_control plugin creates controller_manager inside Gazebo |
| `$(find ...)` not working in URDF | Convert to xacro, use `xacro.process_file()` in launch |
| Robot not visible in Gazebo | WSL2 issue - zoom out or use `export LIBGL_ALWAYS_SOFTWARE=1` |

---

## Updated File Structure

```
harvesting_ws/
├── src/
│   ├── example_arm_description/     # Simple 2-DOF arm (MoveIt + RViz)
│   ├── example_arm_moveit_config/   # MoveIt configuration
│   ├── robot_arm/                   # 4-DOF arm (Gazebo simulation)
│   │   ├── launch/
│   │   │   └── bot.launch.py
│   │   ├── urdf/
│   │   │   └── mybot.urdf.xacro
│   │   ├── yaml/
│   │   │   └── controllers.yaml
│   │   ├── src/
│   │   │   └── write_pos.py
│   │   ├── robot_arm/
│   │   │   └── __init__.py
│   │   ├── resource/
│   │   │   └── robot_arm
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── README.md
│   ├── orchestrator/
│   ├── vision_ml/
│   ├── robotic_actor/
│   └── logger_node/
│
└── docs/
    ├── CHECKPOINT.md
    ├── NOTES.md
    └── RESEARCH/
        └── iris_arm/                # Original source (reference)
```

---

## Current Status (Updated)

- [x] URDF with TCP frame (example_arm)
- [x] ros2_control mock hardware (example_arm)
- [x] MoveIt 2 motion planning (example_arm)
- [x] RViz Plan & Execute works (example_arm)
- [x] arm_commander node with joint-space goals (example_arm)
- [x] **iris_arm converted to ROS2** (robot_arm)
- [x] **Gazebo simulation working** (robot_arm)
- [x] **Joint trajectory control verified** (robot_arm)
- [ ] Test arm_commander movement to targets (example_arm)
- [ ] Integrate with orchestrator

---

## Summary: Two Arm Packages

| Package | DOF | Simulation | Motion Planning |
|---------|-----|------------|-----------------|
| `example_arm_description` + `example_arm_moveit_config` | 2 (slider + arm) | RViz + mock hardware | MoveIt 2 |
| `robot_arm` (iris_arm) | 4 + gripper | Gazebo | Direct trajectory control |
