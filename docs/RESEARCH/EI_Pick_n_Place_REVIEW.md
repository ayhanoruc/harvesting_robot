# EI_Pick_n_Place Repository - Comprehensive Review

## Overview

**Repository**: https://github.com/metanav/EI_Pick_n_Place
**Purpose**: Pick-and-place system using Arduino Braccio++ arm with Edge Impulse YOLOv5 detection
**Hardware**: Arduino Braccio++ (6-DOF), OAK-D camera (depth + RGB), Arduino Nano RP2040 Connect

---

## Architecture Summary

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SYSTEM ARCHITECTURE                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────────┐    ┌──────────────────┐                      │
│  │   OAK-D Camera   │    │  Braccio++ Arm   │                      │
│  │   (DepthAI)      │    │  (6 Servos)      │                      │
│  └────────┬─────────┘    └────────┬─────────┘                      │
│           │                       │                                 │
│           ▼                       ▼                                 │
│  ┌──────────────────┐    ┌──────────────────┐                      │
│  │ YOLOv5 Spatial   │    │ Arduino micro-ROS│                      │
│  │ Detection Node   │    │ Controller       │                      │
│  │ (Edge Impulse)   │    │ (RP2040)         │                      │
│  └────────┬─────────┘    └────────┬─────────┘                      │
│           │                       │                                 │
│           │  /spatial_detections  │  /joint_states (pub)           │
│           │                       │  /follow_joint_trajectory (act)│
│           │                       │  /gripper_cmd (action)         │
│           ▼                       ▼                                 │
│  ┌─────────────────────────────────────────────┐                   │
│  │          pick_n_place.cpp                   │                   │
│  │    (MoveIt Task Constructor Node)           │                   │
│  │                                             │                   │
│  │  • Subscribes to spatial detections         │                   │
│  │  • TF2 transform: camera → base_link        │                   │
│  │  • Updates MoveIt planning scene            │                   │
│  │  • Executes MTC pick-and-place task         │                   │
│  └─────────────────────────────────────────────┘                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 1. Robot Description (Braccio++ 6-DOF)

### Joint Configuration
| Joint | Name | Type | Axis | Limits (rad) | Function |
|-------|------|------|------|--------------|----------|
| 1 | base_joint | revolute | Z | 0.05 - 5.0 | Base rotation (yaw) |
| 2 | shoulder_joint | revolute | X | 1.6 - 4.0 | Shoulder pitch |
| 3 | elbow_joint | revolute | X | 1.0 - 4.6 | Elbow pitch |
| 4 | wrist_pitch_joint | revolute | X | 0.77 - 4.8 | Wrist pitch |
| 5 | wrist_roll_joint | revolute | Z | 0.2 - 5.0 | Wrist roll |
| 6 | gripper_joint | revolute | Y | 2.6 - 3.85 | Gripper open/close |

### Link Lengths (approximate)
- Base height: ~72mm
- Shoulder link: ~125mm
- Elbow link: ~125mm
- Wrist pitch: ~60mm
- Gripper reach: ~30mm

### Key Differences from Our 4-DOF Arm
| Feature | Braccio++ | Our RoboCot |
|---------|-----------|-------------|
| DOF | 6 (5 arm + 1 gripper) | 4 (arm only) |
| Wrist | Pitch + Roll | Roll only |
| Camera Mount | Separate (fixed) | On lower_arm |
| Control | micro-ROS (Arduino) | ros2_control (sim) |

---

## 2. Perception System

### Hardware: OAK-D Camera
- **RGB Camera**: 1080p color for object detection
- **Stereo Depth**: Left + Right mono cameras for spatial localization
- **Neural Compute**: On-device YOLOv5 inference via VPU

### Detection Pipeline (`ei_yolov5_spatial_stream.py`)

```python
# Key components:
1. YoloSpatialDetectionNetwork (on OAK-D VPU)
   - Model: Edge Impulse trained YOLOv5n (320x320)
   - Classes: ["Penguin", "Pig"] (demo objects)
   - Confidence threshold: 0.7
   - IOU threshold: 0.5

2. Stereo Depth Integration
   - Depth range: 200mm - 1000mm
   - Bounding box depth sampling: 25% of box area
   - Returns: spatialCoordinates (X, Y, Z in mm)

3. Output Message
   - depthai_ros_msgs/SpatialDetectionArray
   - Each detection has: class, confidence, bbox, position (3D)
```

### Key Insight: Direct 3D from Detection
The OAK-D's `YoloSpatialDetectionNetwork` returns 3D coordinates **directly** with each detection - no separate depth lookup needed:

```python
# Each detection contains:
detection.spatialCoordinates.x  # mm from camera
detection.spatialCoordinates.y  # mm from camera
detection.spatialCoordinates.z  # depth in mm
```

**Lesson for RoboCot**: Our current approach (separate depth_processor service) is more flexible but adds latency. Consider:
1. Keep current approach for precise targeting
2. For initial detection, YOLO bbox center + average depth in ROI is sufficient

---

## 3. Pick-and-Place Logic (MTC)

### MoveIt Task Constructor (MTC) Pipeline

The system uses MTC's stage-based approach for pick-and-place:

```cpp
// Task stages (in order):
1. CurrentState          - Capture current robot state
2. OpenHand             - Open gripper
3. Connect("move to pick") - Plan path to pre-grasp
4. SerialContainer("pick object"):
   ├── MoveRelative("approach")  - Move toward object
   ├── GenerateGraspPose         - Sample grasp poses around object
   ├── ComputeIK                 - Solve IK for grasp pose
   ├── AllowCollision            - Allow hand-object collision
   ├── CloseHand                 - Close gripper
   ├── AttachObject              - Attach to planning scene
   └── MoveRelative("lift")      - Lift object up
5. Connect("move to place") - Plan path to place location
6. SerialContainer("place object"):
   ├── GeneratePlacePose         - Compute place position (bins)
   ├── ComputeIK                 - Solve IK for place pose
   ├── OpenHand                  - Release object
   ├── ForbidCollision           - Disable hand-object collision
   ├── DetachObject              - Remove from planning scene
   └── MoveRelative("retreat")   - Back away
7. MoveTo("return home")    - Return to ready position
```

### Grasp Frame Computation

Critical insight - the grasp pose is computed relative to object distance:

```cpp
// Grasp approach angle based on distance (curve fitting!)
float d = sqrt(pow(object.pose.position.x, 2) + pow(object.pose.position.y, 2));
float x_angle = 3.85798 * d + (-0.21251);  // Empirically tuned

// Grasp frame transform (gripper orientation)
Eigen::Quaterniond q =
    Eigen::AngleAxisd(x_angle, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());
```

**Lesson**: The grasp angle varies with distance! Objects closer to the base need steeper approach.

### Object Sorting Logic

Objects are sorted into bins based on class:

```cpp
// Bin positions (polar coordinates from base)
#define BIN_DIST 0.38
#define BIN_0_ANGLE  (7.0f * M_PI/18)   // +70° for class "Penguin"
#define BIN_1_ANGLE -(7.0f * M_PI/18)   // -70° for class "Pig"
```

**Parallel to RoboCot**: Replace bins with "reservoir" for cotton deposit.

---

## 4. Coordinate Transforms

### Detection to Planning Scene

```cpp
// 1. Detection comes in camera frame
geometry_msgs::msg::PoseStamped pose_from_cam;
pose_from_cam.header = msg->header;  // "oak_d_link" or similar
pose_from_cam.pose.position = detection.position;
pose_from_cam.pose.position.y = -pose_from_cam.pose.position.y;  // Y flip!

// 2. Transform to base_link using TF2
geometry_msgs::msg::PoseStamped pose_from_base_link =
    tf_buffer_->transform(pose_from_cam, "base_link", tf2::Duration(0));

// 3. Round to nearest degree (for GenerateGraspPose sampling)
float radians = atan(pose.y / pose.x);
float degrees_rounded = round(radians * 180 / M_PI);
```

**Note**: The Y-axis flip is specific to OAK-D's coordinate convention. Our Gazebo camera may differ.

---

## 5. Arduino Controller (micro-ROS)

### Communication Interface

```cpp
// ROS2 Interfaces on Arduino:
- Publisher:   /joint_states (sensor_msgs/JointState)
- Action:      /arm/follow_joint_trajectory (FollowJointTrajectory)
- Action:      /gripper/gripper_cmd (GripperCommand)
- Client:      /trigger_motion_planning (std_srvs/Trigger)
```

### Trajectory Execution

```cpp
void execute_arm_trajectory(trajectory_msgs::msg::JointTrajectory *trajectory) {
    for (int i = 0; i < trajectory->points.size; i++) {
        float base_pos        = RAD_TO_DEG * trajectory->points.data[i].positions.data[0];
        float elbow_pos       = RAD_TO_DEG * trajectory->points.data[i].positions.data[1];
        float shoulder_pos    = RAD_TO_DEG * trajectory->points.data[i].positions.data[2];
        float wrist_pitch_pos = RAD_TO_DEG * trajectory->points.data[i].positions.data[3];
        float wrist_roll_pos  = RAD_TO_DEG * trajectory->points.data[i].positions.data[4];

        Braccio.moveTo(grippper_pos, wrist_roll_pos, wrist_pitch_pos,
                       elbow_pos, shoulder_pos, base_pos);
        delay(50);
    }
}
```

**Key Point**: MoveIt2 sends joint names in **alphabetical order**, not kinematic order. Must map correctly!

---

## 6. Key Learnings for RoboCot

### 6.1 Perception Strategy
| Their Approach | Our RoboCot Equivalent |
|----------------|------------------------|
| OAK-D spatial detection (3D from detector) | depth_processor service (separate lookup) |
| Fixed camera, TF to base | Camera on arm, TF to world |
| Single-frame detection | Multi-frame tracking needed for moving camera |

### 6.2 Motion Planning
| Their Approach | Our RoboCot Equivalent |
|----------------|------------------------|
| MTC full pipeline | Simple MoveGroup action calls |
| GenerateGraspPose (360° sampling) | Direct IK to target |
| 6-DOF full orientation control | 4-DOF limited (position + yaw only) |

### 6.3 Workflow Comparison

**Their Flow**:
```
Button press → Get detections → Update scene → MTC pick → MTC place → Home
```

**Our Proposed Flow**:
```
Home → Scan (sweep) → Detect clusters → camera_focus → Approach cluster →
Per-boll: center → depth → approach → pick → deposit → Next boll
```

### 6.4 Specific Recommendations

1. **Keep depth_processor as-is**: Flexible for different use cases
2. **camera_focus heuristic is good**: Simpler than their full IK approach
3. **Add multi-frame detection**: Track detections across camera motion
4. **Consider MTC for pick sequence**: Cleaner than manual stage orchestration
5. **Grasp angle vs distance**: Implement similar curve-fitting for our gripper approach

---

## 7. Files Reference

### Core Files
```
pnp_ws/src/
├── pick_n_place/
│   └── src/pick_n_place.cpp          # Main orchestrator (MTC)
├── ei_yolov5_detections/
│   └── src/ei_yolov5_spatial_stream.py  # OAK-D + YOLOv5
├── braccio_description/
│   └── urdf/braccio_arm.xacro        # Robot URDF (6-DOF)
└── braccio_moveit_config/
    └── config/braccio.srdf           # MoveIt planning groups

Arduino/
└── braccio_plus_plus_controller_final_v3.1/
    └── *.ino                          # micro-ROS controller
```

### Message Types Used
- `depthai_ros_msgs/SpatialDetectionArray` - Detections with 3D
- `control_msgs/action/FollowJointTrajectory` - MoveIt trajectories
- `control_msgs/action/GripperCommand` - Gripper control
- `sensor_msgs/JointState` - Joint feedback

---

## 8. Conclusion

The EI_Pick_n_Place project provides excellent reference for:
1. **Detection → 3D → Planning Scene** pipeline
2. **MTC stage-based manipulation** (though may be overkill for 4-DOF)
3. **Grasp geometry** calculations (distance-based approach angle)
4. **Physical system integration** (micro-ROS on Arduino)

For RoboCot, we can adopt:
- The overall workflow concept (detect → focus → approach → pick)
- Distance-based grasp angle computation
- Multi-stage task decomposition

---

*Review created: 2026-01-03*
*For: RoboCot Cotton Harvesting Robot Project*


---

## 9. Adaptation Guide (From Edge Impulse Docs)

**Source**: [Edge Impulse - ROS 2 Pick and Place System](https://docs.edgeimpulse.com/projects/expert-network/robotic-arm-sorting-arduino-braccio)

> "The system can be adapted to different scenarios by changing the camera, the robot arm, the gripper, or the software."

### 9.1 What You CAN Swap Out

| Component | Their Setup | RoboCot Equivalent | Effort |
|-----------|-------------|-------------------|--------|
| **Camera** | OAK-D (DepthAI) | Gazebo RGB-D camera | Already done |
| **Robot Arm** | Braccio++ (6-DOF) | Custom 4-DOF arm | Already done |
| **Gripper** | Braccio gripper | Custom cotton gripper | TBD |
| **Detection Model** | Edge Impulse YOLOv5 | Your YOLO9 (cotton) | Ready to integrate |
| **ROS2 Framework** | Humble on RPi5 | Jazzy on Windows | Already done |

### 9.2 Adaptation Steps (Applied to RoboCot)

#### Camera Adaptation
```
Original: OAK-D → DepthAI ROS → /spatial_detections
RoboCot:  Gazebo RGB-D → image topics → YOLO node + depth_processor
```
- They get 3D coords directly from OAK-D spatial detection
- We get 3D coords via: YOLO bbox → depth_processor service
- **Same result, different path**

#### Robot Arm Adaptation
```
Original: Braccio++ URDF → MoveIt Setup Assistant → braccio_moveit_config
RoboCot:  Custom URDF → MoveIt Setup Assistant → robot_arm_moveit_config
```
- Change URDF → regenerate MoveIt config
- Update planning groups in SRDF
- Adjust joint limits and link names
- **Already done for our arm**

#### Gripper Adaptation
```
Original: Braccio gripper (mimic joint) → /gripper_cmd action
RoboCot:  Cotton gripper (TBD) → gripper controller
```
- Define gripper in URDF (separate planning group)
- Add gripper controller to ros2_control
- Create open/close named poses in SRDF

#### Detection Model Adaptation
```
Original: Edge Impulse → YOLOv5 → .blob (Myriad VPU)
RoboCot:  Your training → YOLO9 → .pt (PyTorch/Ultralytics)
```
- No conversion needed - use yolo_ros or ultralytics directly
- Publish detections to custom topic
- Orchestrator subscribes and calls depth_processor

### 9.3 Key Insight: The Modular Architecture

Their architecture is **intentionally modular**:

```
┌─────────────────────────────────────────────────────────────┐
│                    SWAPPABLE COMPONENTS                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐   │
│  │   CAMERA    │     │    MODEL    │     │   ROBOT     │   │
│  │  OAK-D      │     │  YOLOv5     │     │  Braccio++  │   │
│  │  RealSense  │     │  YOLOv8     │     │  UR5        │   │
│  │  ZED        │     │  YOLOv9     │     │  Custom     │   │
│  │  Gazebo     │ ←── │  Custom     │ ──→ │  4-DOF      │   │
│  └─────────────┘     └─────────────┘     └─────────────┘   │
│         │                   │                   │           │
│         ▼                   ▼                   ▼           │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              STANDARD ROS2 INTERFACES               │   │
│  │  • sensor_msgs/Image                                │   │
│  │  • vision_msgs/Detection3DArray (or custom)         │   │
│  │  • control_msgs/FollowJointTrajectory               │   │
│  │  • sensor_msgs/JointState                           │   │
│  └─────────────────────────────────────────────────────┘   │
│                            │                                │
│                            ▼                                │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              ORCHESTRATION LAYER                    │   │
│  │  • MoveIt Task Constructor (or custom)              │   │
│  │  • pick_n_place.cpp (or Python equivalent)          │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```
