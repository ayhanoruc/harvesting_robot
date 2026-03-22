# ROBOCOB Integration Analysis for RoboCot

**Date:** 2026-03-19
**Author:** Ayhan (via Claude analysis)
**Purpose:** Evaluate ROBOCOB education robot for RoboCot cotton harvesting project integration

---

## 1. What is ROBOCOB?

ROBOCOB is an **education/research robot** by InoRobotics (inorobotics.com), consisting of 3 main components:

| Component | Model | Key Specs |
|-----------|-------|-----------|
| Mobile Base | INOTA 250 | Differential drive, 250kg payload, Hokuyo UAM-05LP lidar, RealSense D435, Jetson AGX Xavier 64GB |
| Manipulator | **Doosan Robotics M1013** | 6-DOF, 10kg payload, 1300mm reach, 33kg weight, 1m/s max speed |
| Gripper | **Robotiq Hand-E** | Parallel jaw, 50mm stroke, 7kg payload, RS485 comms, position/speed/force control |

**Software Stack:** Ubuntu 20.04, **ROS Noetic** (ROS1, NOT ROS2), MoveIt1, Gazebo Classic

---

## 2. What We Get vs What We Planned

### 2.1 Manipulator Comparison: Doosan M1013 vs Arduino Braccio++

| Spec | Braccio++ (planned) | Doosan M1013 (available) | Impact |
|------|---------------------|--------------------------|--------|
| DOF | 6 | 6 | Same |
| Reach | ~520mm | **1300mm** | 2.5x more reach -- massive upgrade |
| Payload | ~0.8kg (est.) | **10kg** | Industrial grade |
| Weight | ~0.8kg | 33kg | Much heavier, needs fixed base or mobile platform |
| Repeatability | Unknown (hobby servo) | **+/- 0.05mm** (industrial) | Orders of magnitude better |
| Max Speed | ~30 deg/s (servo limit) | ~1m/s TCP | Much faster |
| Control | Arduino PWM -> ros2_control | **Doosan controller + ROS driver** | Professional controller included |
| Joint Limits | See report Table 20 | J1-J6 all +/-360 (SW limited by TP) | More flexible |
| Price | ~14.5K TRY | Already available (university owned) | **Free** |

### 2.2 Gripper Comparison: Robotiq Hand-E vs Our PDS

| PDS Requirement | Robotiq Hand-E | Status |
|-----------------|---------------|--------|
| SR-04: 50-60mm opening | 50mm parallel stroke | PASS |
| QL-05: 90% pick rate | Industrial adaptive gripper, force/position/speed control | Likely PASS |
| MF-01: 3D printable | Not needed -- industrial gripper provided | N/A (better) |
| Force control | 0-255 configurable, re-grasp feature | EXCEEDS |
| Object detection | gOBJ status (detected while opening/closing) | BONUS |

### 2.3 What We DON'T Get

- **No ZED X Mini camera** -- ROBOCOB has RealSense D435 on the mobile base, but NOT on the arm end-effector
- **No Jetson Orin NX** -- ROBOCOB uses Jetson AGX Xavier (actually more powerful, but it's shared with the mobile base system)
- **No Arduino UNO R4 WiFi** -- Doosan has its own controller

---

## 3. Critical Architecture Differences

### 3.1 ROS1 (Noetic) vs ROS2 (Humble)

**This is the biggest integration challenge.** Our entire codebase is ROS2 Humble + Gazebo Ignition Fortress. ROBOCOB runs ROS1 Noetic + Gazebo Classic.

**Options:**
1. **Port our pipeline to ROS1** -- Significant effort but gives native ROBOCOB compatibility
2. **Use ros1_bridge** -- Run both ROS1 and ROS2 nodes, bridge topics between them
3. **Port ROBOCOB drivers to ROS2** -- Doosan officially supports ROS2 (check their GitHub), Robotiq also has ROS2 packages
4. **Hybrid approach** -- Keep Doosan/Robotiq on ROS1, bridge only essential topics (joint_states, cmd_vel, gripper commands)

**Recommendation:** Option 3 or 4. Doosan has ROS2 packages available. The Robotiq Hand-E can be controlled via Modbus/RS485 directly.

### 3.2 Communication Architecture

ROBOCOB's INOTA 250 mobile base uses **MQTT** (not standard ROS topics) for low-level communication:
- ACU card communicates via MQTT over ethernet (broker at 192.168.3.4:1883)
- `robot_communication` package bridges MQTT <-> ROS topics
- Heartbeat system: if heartbeat stops, robot auto-stops (safety)
- STO (Safe Torque Off) active by default on startup

**For our project:** We likely won't use the mobile base for cotton harvesting (fixed-base scenario). But if we do, we need the MQTT bridge running.

### 3.3 Network Architecture

All devices on 192.168.3.x subnet:
- Robot PC (Jetson AGX Xavier): 192.168.3.4
- ACU control card: 192.168.3.3
- Doosan M1013 controller: **192.168.3.5** (port 12345)
- Hokuyo lidar: 192.168.3.7

---

## 4. Doosan M1013 -- Detailed Technical Notes

### 4.1 Controller Types
- **AC Controller:** Main power, connected to wall power. Has teach pendant, I/O terminals, robot cable connection.
- **DC Controller:** Battery powered option. Same functionality, different power source.

### 4.2 Teach Pendant (El Kumandasi)
- Touch screen interface with Jog mode for manual positioning
- Safety features: E-stop button, direct teach button (free-drive mode)
- **Important:** To switch from pendant to ROS control, must run `switch_to_ros_control.py` and approve on pendant

### 4.3 ROS Control Commands

```bash
# Virtual mode (simulation)
roslaunch dsr_launcher single_robot_gazebo.launch mode:=virtual model:=m1013

# Real mode
roslaunch dsr_launcher single_robot_gazebo.launch mode:=real host:=192.168.3.5 port:=12345

# With MoveIt
roslaunch dsr_control dsr_moveit.launch model:=m1013 host:=192.168.3.5 port:=12345 mode:=real color:=blue

# Transfer control from pendant to ROS
rosrun dsr_control switch_to_ros_control.py
```

### 4.4 Joint Specifications

| Joint | Range | Notes |
|-------|-------|-------|
| J1 | +/-360 (TP: +/-360) | Base rotation |
| J2 | +/-360 (TP: +/-95) | Shoulder |
| J3 | +/-160 (TP: +/-135) | Elbow |
| J4 | +/-360 (TP: +/-360) | Wrist 1 |
| J5 | +/-360 (TP: +/-135) | Wrist 2 |
| J6 | +/-360 (TP: +/-360) | Wrist 3 (flange) |

### 4.5 Servo On/Off
- **Servo ON:** Joints powered, robot ready to move
- **Servo OFF:** Joints unpowered (triggered by E-stop or safety violation)
- Must be Servo ON before any motion command

---

## 5. Robotiq Hand-E -- Detailed Technical Notes

### 5.1 Control Parameters

| Parameter | Range | Description |
|-----------|-------|-------------|
| Position (rPR) | 0-255 | 0 = fully open (50mm), 255 = fully closed |
| Speed (rSP) | 0-255 | Movement speed |
| Force (rFR) | 0-255 | Grip force (0 = fragile/no re-grasp, 1-50 = fragile with re-grasp, 101-150 = normal, 201-255 = max) |

### 5.2 Activation Parameters
- **rACT:** 0=reset/deactivate, 1=activate
- **rGTO:** 0=stop, 1=go to position
- **rATR:** Auto-release mode
- **rARD:** Auto-release direction

### 5.3 Status Feedback
- **gACT:** Activation status
- **gGTO:** Go-to status
- **gSTA:** 0=reset, 1=activating, 3=activation complete
- **gOBJ:** 0=moving, 1=object detected opening, 2=object detected closing, 3=at requested position

### 5.4 ROS Interface

```bash
# Launch gripper driver
roslaunch robotiq_hande_driver_noetic robotiq_hande_driver_modular.launch

# Set position via service
rosservice call /gripper_cmd/set_position "position: 0.026, speed: 200, force: 150"

# Activate gripper
rosservice call /gripper_cmd/activate

# Topics
/gripper_cmd/status          # Gripper status
/gripper_cmd/joint_states    # For MoveIt
/gripper_cmd/command          # Send commands
```

### 5.5 Connection
- RS485 via USB converter -> robot's USB hub
- Port fixed to `/dev/robotiq_gripper` via udev rules (install_gripper.sh)
- Baudrate: 115200, Slave ID: 9

---

## 6. MoveIt Configuration (robocob_moveit package)

### 6.1 Planning Groups
- **m1013:** 6 revolute joints (joint1-joint6), chain from `support_cube` to `tool0`
- **hande:** 2 prismatic joints (left_finger, right_finger), fixed base joint

### 6.2 Controllers
- `robocob_moveit/dsr_joint...`: JointTrajectoryController for arm (joint1-6)
- `gripper_cmd`: GripperActionController for Hand-E (left/right finger)

### 6.3 Launch
```bash
# Full system with all components
roslaunch robocob_moveit robocob_master.launch

# Parameters:
# include_mobile_base: true/false
# include_arm: true/false
# include_gripper: true/false (requires include_arm=true)
# use_moveit: true/false
# use_navigation: true/false
```

### 6.4 RViz Configs
- `mobile_base_moveit.rviz` -- mobile robot only
- `mobile_base_moveit_w_nav.rviz` -- full system with navigation
- `non_mobile_base_moveit.rviz` -- arm + gripper only (most relevant for us)

---

## 7. Simulation

```bash
# Launch Gazebo simulation with full robot
roslaunch robot_description moveit_gazebo.launch

# Parameters available:
# gui: true/false
# world: gripper_lab.world
# include_mobile_base: true/false
# include_arm: true/false
# include_gripper: true/false
# include_navigation: true/false
```

Example pick-and-place code exists:
```bash
rosrun robocob_navigation navigate_and_pick.py
```
This demonstrates a full 12-step navigate -> pick -> place sequence using move_base + MoveIt + gripper actions.

---

## 8. Integration Plan for RoboCot

### 8.1 Phase 1: Get Doosan + Hand-E Working in Our Sim (Priority 1)

1. **Check Doosan ROS2 support** -- Doosan provides official ROS2 packages. Clone and test with our ROS2 Humble + Gazebo Ignition setup.
2. **Get M1013 URDF/xacro** -- Extract from ROBOCOB's `robot_description` package or Doosan's official repo.
3. **Replace Braccio URDF** with M1013 in our simulation. This means:
   - Update `robot_arm` package URDF
   - Reconfigure `robot_arm_moveit_config` for M1013
   - Update joint names in `orchestrator` nodes (explorer, camera_focus, etc.)
   - Recalibrate panoramic scan angles for M1013's kinematics
4. **Mount RGB-D camera on M1013 end-effector** -- Add camera to tool0 flange in URDF (eye-in-hand config, same as current design)
5. **Integrate Hand-E gripper** -- Either use Robotiq ROS2 packages or port the `robotiq_hande_gripper_noetic` driver

### 8.2 Phase 2: Adapt Vision Pipeline

1. Camera intrinsics will change if we use a different camera (RealSense D435 vs simulated camera vs ZED X Mini)
2. Update `depth_processor.py` K matrix values
3. Re-tune `camera_focus.py` gains for M1013's different kinematic structure
4. Panoramic scan grid needs redesign -- M1013's 1300mm reach means the scanning geometry changes significantly
5. Mock field dimensions may need adjustment (clusters can be further away now)

### 8.3 Phase 3: End-Effector Control & Collection Routine

1. **Gripper control node:** Write ROS2 wrapper for Hand-E (position/speed/force commands)
2. **Pick sequence:** Approach -> Close gripper -> Check gOBJ status -> Lift -> Transfer to reservoir -> Open gripper
3. **Force control tuning:** Determine optimal force (0-255) for cotton boll grasping -- cotton is delicate, probably 50-100 range
4. **Re-grasp strategy:** If gOBJ=0 after close (no object detected), retry or move to next target

### 8.4 Phase 4: Hardware Deployment

1. Set up Doosan M1013 with controller (AC or DC)
2. Connect Hand-E via RS485-USB
3. Mount camera on tool0 flange
4. Configure network (192.168.3.x)
5. Run `switch_to_ros_control.py` to transfer control from pendant
6. Deploy our cotton detection + picking pipeline

---

## 9. Key Advantages Over Original Plan

| Aspect | Original (Braccio) | New (Doosan M1013) |
|--------|-------------------|-------------------|
| Reach | 520mm (barely covers mock field) | 1300mm (massive workspace) |
| Repeatability | sigma=7.4mm (sim), unknown real | **+/- 0.05mm** (industrial spec) |
| QL-02 spec (+/-3mm) | FAIL in sim | Almost certainly PASS |
| Payload | ~0.8kg (camera only) | 10kg (camera + tools + margin) |
| Gripper | Custom 3D printed | Industrial Robotiq Hand-E with force feedback |
| Controller | Arduino PWM -> ROS bridge | Professional Doosan controller |
| Safety | Software-only E-stop | Hardware E-stop + teach pendant + safety zones |
| Cost | ~157K TRY (needed procurement) | **Already available** |

---

## 10. Key Risks & Challenges

1. **ROS1 vs ROS2 gap:** Biggest technical challenge. Need to either bridge or port.
2. **No dedicated compute for ML:** AGX Xavier is shared with mobile base. May need to run YOLO on a separate machine or the Xavier directly.
3. **Camera mounting:** M1013's tool0 flange uses industrial connectors. Need adapter plate for whatever camera we use.
4. **Mobile base not needed initially:** For fixed-base cotton picking demo, we can use `include_mobile_base=false`. But the arm still needs the controller.
5. **Teach pendant dependency:** Switching to ROS control requires physical access to the pendant each time.
6. **Weight:** 33kg arm + controller is not portable like Braccio. Lab setup only.
7. **Learning curve:** Doosan's API and control flow is more complex than simple joint trajectory commands.

---

## 11. GitHub Repositories to Check

The manual references two repos:
1. `https://github.com/inomuh/robocob_ws_sim` -- Simulation workspace (private repo, need access from InoRobotics)
2. Doosan official ROS packages -- Check for ROS2 Humble support

**Action items:**
- [ ] Request access to ROBOCOB GitHub repos from instructor/InoRobotics
- [ ] Check Doosan Robotics official GitHub for ROS2 packages
- [ ] Check Robotiq GitHub for ROS2 Hand-E drivers
- [ ] Get physical access to the robot and pendant

---

## 12. Quick Reference: Essential Commands

```bash
# SSH into robot
ssh robocob@192.168.3.4  # password: 1

# Start system service
sudo service robocob start|stop|restart|status

# Launch real robot with MoveIt
roslaunch dsr_control dsr_moveit.launch model:=m1013 host:=192.168.3.5 port:=12345 mode:=real

# Transfer control to ROS
rosrun dsr_control switch_to_ros_control.py

# Gripper
roslaunch robotiq_hande_driver_noetic robotiq_hande_driver_modular.launch
rosservice call /gripper_cmd/set_position "position: 0.0, speed: 200, force: 150"  # fully open
rosservice call /gripper_cmd/set_position "position: 0.025, speed: 200, force: 100" # close

# Full system
roslaunch robocob_moveit robocob_master.launch include_mobile_base:=false

# Simulation
roslaunch robot_description moveit_gazebo.launch include_mobile_base:=false
```

---

## 13. Summary

The Doosan M1013 + Robotiq Hand-E is a **massive upgrade** over the planned Braccio++. We get industrial-grade repeatability (0.05mm vs our 7.4mm sigma), 2.5x reach, 10kg payload, professional gripper with force feedback, and it's already available at no cost.

The main challenge is the **ROS1 vs ROS2 gap**. Our best path forward is:
1. Get access to the hardware and repos ASAP
2. Check if Doosan has ROS2 Humble drivers (likely yes)
3. Adapt our simulation to use M1013 URDF
4. Port our YOLO + spatial detection pipeline to work with the new arm kinematics
5. Implement the pick-and-place routine using Hand-E's force-controlled grasping

This change fundamentally de-risks the hardware side of the project. The QL-02 repeatability spec that we FAILED in simulation should now be trivially achievable with the Doosan's 0.05mm repeatability.
