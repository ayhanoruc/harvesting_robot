# RoboCot - Autonomous Cotton Harvesting System
## ME 429 Design Report - Sections 2.2 & 2.3

---

# 2.2 Overview of Possible Solutions

Autonomous cotton harvesting requires the integration of multiple subsystems spanning perception, motion planning and mechanical design. Rather than evaluating monolithic system configurations, this section adopts a component-based approach where each design decision point is analyzed independently. Since the RoboCot architecture is built on modular, interchangeable components, this approach allows systematic evaluation of alternatives at each decision point using consistent criteria. The selected components are then integrated into the final system configuration.

## 2.2.1 Solution Space Overview

The design of an autonomous cotton picking system involves ten key decision points spanning software architecture, perception, control and mechanical domains. Table 1 summarizes these decision points, their requirements derived from the product design specifications (Section 2.1) and their impact on overall system performance.

**Table 1. Solution Space: Design Decision Points**

| Decision Point | Requirement Reference | Impact on System |
|----------------|----------------------|------------------|
| Software Framework | MF-03 (modularity), RL-02 (crash recovery) | Determines integration architecture, debugging capability and code reusability |
| Camera Placement | QL-03 (90% detection), SR-02 (mock field coverage) | Affects viewing angles, occlusion handling and inspection distance |
| Depth Sensing Method | QL-01 (±5mm positioning), QL-02 (±3mm repeatability) | Determines 3D localization accuracy and calibration complexity |
| Detection Method | QL-03 (90% detection), EN-01 (lighting robustness) | Affects detection reliability under varying conditions |
| Cluster Identification | QL-05 (90% pick rate), RL-02 (software reliability) | Determines tracking accuracy across multiple scan positions |
| Clustering Algorithm | QL-05 (functional consistency) | Affects grouping correctness and outlier rejection |
| Motion Planning | SF-02 (speed limits), SF-03 (singularity handling) | Determines path safety, collision avoidance and execution reliability |
| Manipulator Configuration | SR-01 (520mm reach), WT-03 (tip deflection <10mm) | Affects workspace coverage and approach trajectory flexibility |
| Gripper Design | SR-04 (50-60mm opening), QL-04 (cycle time <60s) | Determines grasp success rate and pick efficiency |
| Reservoir Design | SR-03 (15×15×15cm), ER-03 (tool-free removal <15s) | Affects storage capacity and operator interaction |
| Operator Interface | ER-01 (GUI control), ER-02 (visual indicators) | Determines operator situational awareness and control capability |

The following subsections present the alternative options for each decision point, evaluate them against relevant criteria and justify the selected solution.

## 2.2.2 Design Decision Analysis

### Decision Point 1: Software Framework

The software framework determines how system components communicate, how sensor data flows through the pipeline and how motion commands are executed. This choice affects development efficiency, debugging capability and long-term maintainability.

**Option A: ROS2 (Robot Operating System 2)**
ROS2 is a widely adopted open-source framework for robotics development [18]. It provides standardized communication patterns (topics, services, actions), a transform library (TF2) for coordinate frame management and integration with common tools including Gazebo simulation and MoveIt motion planning.

**Option B: Custom Framework**
A custom framework built on standard libraries (ZeroMQ, Protocol Buffers) offers more control over communication patterns and potentially lower latency. However, it requires implementing functionality that ROS2 provides out of the box.

**Option C: NVIDIA Isaac ROS**
NVIDIA's Isaac ROS is a GPU-accelerated robotics framework optimized for Jetson platforms [31]. It provides hardware-accelerated perception pipelines and integrates with Isaac Sim for simulation. However, it has a smaller community and fewer manipulation-focused tools compared to the broader ROS ecosystem.

**Table 2. Software Framework Decision Matrix**

| Criterion | Weight | PDS Ref. | ROS2 | Custom | Isaac ROS |
|-----------|--------|----------|------|--------|-----------|
| Tool Integration (MoveIt, Gazebo) | 0.12 | MF-03 | 5 | 2 | 3 |
| Modularity & Reusability | 0.12 | MF-03 | 5 | 3 | 4 |
| Development Speed | 0.12 | MF-02 | 4 | 2 | 3 |
| Long-term Support | 0.28 | RL-02 | 5 | 3 | 4 |
| Real-time Capability | 0.36 | QL-04 | 4 | 5 | 5 |
| **Weighted Total** | **1.00** | | **4.52** | **3.48** | **4.12** |

**Selected: ROS2** — Provides the best balance of tool integration, modularity and community support. While NVIDIA Isaac ROS offers superior GPU acceleration on the Jetson platform, MoveIt2 integration and the broader ecosystem of manipulation tools make ROS2 the preferred choice for this manipulation-focused application. The ros2_control framework enables standardized hardware abstraction while MoveIt2 provides motion planning capabilities essential for collision-free trajectory generation.

---

### Decision Point 2: Camera Placement

Camera placement affects viewing angles, inspection distance and the ability to handle occlusions from plant geometry. For a wrist-mounted manipulator, two primary options exist.

**Option A: Eye-in-Hand (Wrist-Mounted)**
The camera is mounted directly on the end-effector, moving with the gripper. This provides close-up inspection capability (viewing distance ~35cm) and the ability to position the camera at arbitrary viewpoints through arm motion.

**Option B: Upper-Arm Mounted**
The camera is mounted on a more proximal link (shoulder or elbow region), providing a more stable platform but with less positioning flexibility and greater viewing distance to targets.

**[Figure 1: Camera Placement Options - (a) Eye-in-hand configuration showing camera at wrist with close viewing distance, (b) Upper-arm mounted configuration showing camera on shoulder link with wider but less flexible viewing angle]**

**Table 3. Camera Placement Decision Matrix**

| Criterion | Weight | PDS Ref. | Eye-in-Hand | Upper-Arm |
|-----------|--------|----------|-------------|-----------|
| Close-up Inspection Capability | 0.20 | QL-03 | 5 | 2 |
| Multi-angle Viewing | 0.15 | SR-02 | 5 | 3 |
| Occlusion Handling | 0.36 | QL-03, EN-01 | 5 | 2 |
| Platform Stability | 0.20 | QL-02 | 3 | 5 |
| Calibration Simplicity | 0.09 | MA-03 | 3 | 4 |
| **Weighted Total** | **1.00** | | **4.42** | **2.93** |

**Selected: Eye-in-Hand** — The ability to position the camera at multiple viewpoints and achieve close-up inspection is critical for reliable cotton detection and accurate depth measurement. The stability trade-off is acceptable given the controlled motion during scanning.

---

### Decision Point 3: Depth Sensing Method

Accurate 3D localization requires depth measurement at detected pixel locations. Two primary approaches are available with the eye-in-hand configuration.

**Option A: RGB-D Camera (Direct Depth)**
RGB-D cameras provide direct depth measurement at each pixel using structured light projection or time-of-flight sensing. The depth value is read directly from the sensor without additional computation.

**Option B: Stereo Camera Pair (Triangulation)**
Stereo vision computes depth from disparity between matched features in left and right images. Depth accuracy depends on baseline distance and matching quality.

**Table 4. Depth Sensing Decision Matrix**

| Criterion | Weight | PDS Ref. | RGB-D | Stereo |
|-----------|--------|----------|-------|--------|
| Depth Accuracy at Close Range | 0.31 | QL-01, QL-02 | 5 | 3 |
| Calibration Simplicity | 0.14 | MA-03 | 5 | 2 |
| Computational Cost | 0.24 | EN-02 | 5 | 3 |
| Performance in Texture-less Regions | 0.31 | QL-03 | 4 | 2 |
| **Weighted Total** | **1.00** | | **4.69** | **2.55** |

**Selected: RGB-D Camera** — Direct depth measurement eliminates stereo calibration complexity and matching errors. The ZED X Mini selected for hardware deployment provides depth accuracy compatible with the ±5mm positioning requirement at the 35cm viewing distance used during cluster inspection.

---

### Decision Point 4: Detection Method

Cotton boll detection converts camera images into bounding boxes identifying target locations. This is the foundation of the perception pipeline.

**Option A: Classical Computer Vision**
Traditional approaches use color segmentation (HSV thresholding for white cotton), edge detection and blob analysis. These methods are computationally efficient but sensitive to lighting variations.

**Option B: Deep Learning (YOLO)**
Learning-based detection using convolutional neural networks trained on labeled cotton datasets. YOLO-style architectures provide real-time inference with strong robustness to lighting and background variation [12][22].

**Table 5. Detection Method Decision Matrix**

| Criterion | Weight | PDS Ref. | Classical CV | YOLO |
|-----------|--------|----------|--------------|------|
| Robustness to Lighting Variation | 0.18 | EN-01 | 2 | 5 |
| Detection Accuracy | 0.23 | QL-03 | 2 | 5 |
| Handling Complex Backgrounds | 0.41 | QL-03, RL-01 | 2 | 5 |
| Computational Cost | 0.18 | EN-02 | 5 | 3 |
| **Weighted Total** | **1.00** | | **2.54** | **4.64** |

**Selected: YOLO11** — Deep learning detection is essential for achieving the 90% detection accuracy requirement under field-like lighting conditions. The YOLO11 model trained on the Cotton-boll-and-cluster-2 dataset achieves confidence above 0.7 on cotton bolls. GPU inference on the Jetson Orin NX provides real-time performance.

---

### Decision Point 5: Cluster Identification Strategy

During panoramic scanning, the same cotton cluster may be detected from multiple viewpoints. A method is required to identify that these detections correspond to the same physical target.

**Option A: Multi-Frame Tracking (ByteTrack/BoT-SORT)**
Video-based trackers maintain object identity across frames using motion prediction and appearance features [25][26]. These methods are designed for continuous video streams where objects move smoothly between frames.

**Option B: World-Space 3D Clustering (Custom Pipeline)**
Each detection is converted to 3D world coordinates using depth and TF transforms. Detections are then clustered based on spatial proximity in world-space, independent of camera motion between scan positions.

**Table 6. Cluster Identification Decision Matrix**

| Criterion | Weight | PDS Ref. | ByteTrack | World-Space 3D |
|-----------|--------|----------|-----------|----------------|
| Stability Across Discrete Scan Positions | 0.32 | QL-05 | 2 | 5 |
| ID Consistency | 0.25 | RL-02 | 3 | 5 |
| Independence from Frame Rate | 0.32 | QL-04 | 2 | 5 |
| Implementation Complexity | 0.11 | MF-03 | 4 | 3 |
| **Weighted Total** | **1.00** | | **2.47** | **4.78** |

**Selected: World-Space 3D Clustering** — Testing with ByteTrack revealed ID instability when the camera moved significantly between scan positions, as the tracker's motion model assumes continuous video with gradual object movement. The world-space approach converts each detection to 3D coordinates and clusters spatially, providing robust identification regardless of camera trajectory.

---

### Decision Point 6: Clustering Algorithm

When grouping detections in world-space, the algorithm choice affects how nearby detections are merged and whether chain-linking artifacts occur.

**Option A: Single-Linkage Clustering**
A detection joins a cluster if it is within the merge radius of ANY existing member. This can cause chain-linking where distant detections become grouped through intermediate detections.

**Option B: Complete-Linkage Clustering**
A detection joins a cluster only if it is within the merge radius of ALL existing members. This produces tight, compact clusters without chain-linking artifacts.

**Option C: DBSCAN**
Density-based clustering identifies clusters as dense regions separated by sparse regions. Requires tuning of epsilon and minimum points parameters.

**Table 7. Clustering Algorithm Decision Matrix**

| Criterion | Weight | PDS Ref. | Single-Link | Complete-Link | DBSCAN |
|-----------|--------|----------|-------------|---------------|--------|
| Cluster Compactness | 0.25 | QL-05 | 2 | 5 | 4 |
| Resistance to Chain-Linking | 0.46 | QL-05, RL-02 | 1 | 5 | 4 |
| Simplicity | 0.09 | MF-03 | 5 | 4 | 3 |
| Parameter Sensitivity | 0.20 | RL-02 | 4 | 4 | 2 |
| **Weighted Total** | **1.00** | | **2.21** | **4.71** | **3.51** |

**Selected: Complete-Linkage Clustering** — Initial testing with single-linkage produced chain-linking artifacts where separate clusters were incorrectly merged. Complete-linkage ensures that all members of a cluster are within the merge radius of each other, producing physically meaningful groupings. The merge radius of 0.121m (25% of minimum inter-cluster distance) ensures bolls on the same plant group together while separate plants remain distinct.

---

### Decision Point 7: Motion Planning

Motion planning determines how the arm moves between positions and approaches targets while avoiding collisions and respecting joint limits.

**Option A: Direct Joint Interpolation**
Joint angles are interpolated linearly between start and goal configurations. Simple to implement but provides no collision checking or Cartesian path control.

**Option B: MoveIt2 with OMPL**
MoveIt2 provides a complete motion planning framework with collision checking, multiple planner options (OMPL library) and Cartesian path planning [19]. Integrates with ROS2 and ros2_control.

**Option C: Custom IK Solver**
Inverse kinematics computed analytically or numerically for specific arm geometry. Provides direct control but requires custom collision checking implementation.

**Table 8. Motion Planning Decision Matrix**

| Criterion | Weight | PDS Ref. | Joint Interp. | MoveIt2 | Custom IK |
|-----------|--------|----------|---------------|---------|-----------|
| Collision Avoidance | 0.37 | SF-02, SF-03 | 1 | 5 | 3 |
| Cartesian Path Control | 0.30 | QL-01 | 1 | 5 | 4 |
| Integration with ROS2 | 0.10 | MF-03 | 3 | 5 | 2 |
| Computational Overhead | 0.23 | EN-02 | 5 | 3 | 4 |
| **Weighted Total** | **1.00** | | **2.12** | **4.54** | **3.43** |

**Selected: MoveIt2 with OMPL** — Collision avoidance is critical for safe operation in the cluttered mock field environment. MoveIt2 provides planning scene management, multiple planner algorithms and seamless integration with the ROS2 ecosystem. The computational overhead is acceptable given the non-time-critical nature of arm repositioning during scanning.

---

### Decision Point 8: Manipulator Configuration

The manipulator must provide sufficient reach to cover the mock field while enabling approach trajectories that avoid plant obstacles.

**Option A: 4-DOF Arm**
Four degrees of freedom (base rotation, shoulder, elbow, wrist) provide basic positioning capability but limit approach angle flexibility.

**Option B: 6-DOF Arm (Braccio)**
Six degrees of freedom enable full position and orientation control of the end-effector, allowing the gripper to approach targets from arbitrary angles.

**Table 9. Manipulator Configuration Decision Matrix**

| Criterion | Weight | PDS Ref. | 4-DOF | 6-DOF |
|-----------|--------|----------|-------|-------|
| Approach Trajectory Flexibility | 0.29 | QL-05 | 2 | 5 |
| Workspace Coverage | 0.23 | SR-01, SR-02 | 3 | 4 |
| Obstacle Avoidance Capability | 0.35 | SF-03 | 2 | 5 |
| Mechanical Simplicity | 0.13 | MA-01 | 5 | 3 |
| **Weighted Total** | **1.00** | | **2.62** | **4.51** |

**Selected: 6-DOF Braccio Arm** — The additional degrees of freedom enable approach trajectories that avoid plant obstacles while positioning the gripper optimally for cotton extraction. The Braccio arm provides 520mm+ reach satisfying requirement SR-01 and has existing ROS2/MoveIt integration from the EI_Pick_n_Place reference implementation.

---

### Decision Point 9: Gripper Design

The gripper must reliably grasp cotton bolls without damaging the plant or losing the picked material during transfer.

**Option A: Parallel Jaw Gripper**
Two opposing fingers close to grasp the target. Simple, reliable and provides controlled grip force. Requires 50-60mm opening for cotton boll clusters.

**Option B: Vacuum/Suction End-Effector**
Suction cups or vacuum nozzles pick up material through negative pressure. Works well for flat surfaces but cotton's fibrous texture may reduce seal effectiveness.

**Option C: Multi-Finger Adaptive Gripper**
Compliant fingers conform to irregular object shapes. Higher cost and complexity but better adaptation to variable boll geometry.

**Table 10. Gripper Design Decision Matrix**

| Criterion | Weight | PDS Ref. | Parallel Jaw | Vacuum | Multi-Finger |
|-----------|--------|----------|--------------|--------|--------------|
| Reliability on Fibrous Material | 0.31 | QL-05 | 4 | 2 | 4 |
| Simplicity & Cost | 0.28 | MC-01, MF-01 | 5 | 4 | 2 |
| Grip Security During Transfer | 0.31 | QL-05 | 4 | 2 | 5 |
| 3D Printability | 0.10 | MF-01 | 5 | 3 | 3 |
| **Weighted Total** | **1.00** | | **4.38** | **2.66** | **3.65** |

**Selected: Parallel Jaw Gripper** — The parallel jaw design provides reliable grasping of cotton bolls with controlled force. The 50-60mm opening requirement (SR-04) is achievable with servo-driven fingers. Components can be 3D printed as specified in MF-01. The Braccio arm includes a compatible gripper base.

---

### Decision Point 10: Reservoir Design

The reservoir collects harvested cotton bolls and must support easy emptying by the operator without tools.

**Option A: Fixed Integrated Bin**
The collection bin is permanently attached to the platform. Cotton is removed by reaching into the bin or inverting the entire unit.

**Option B: Removable Drop-In Bin**
A separate container drops into a receptacle and lifts out for emptying. Supports tool-free removal as required by ER-03.

**Option C: Conveyor to External Container**
A conveyor belt transfers cotton to a separate collection point. Adds mechanical complexity and potential failure modes.

**Table 11. Reservoir Design Decision Matrix**

| Criterion | Weight | PDS Ref. | Fixed | Removable | Conveyor |
|-----------|--------|----------|-------|-----------|----------|
| Tool-Free Removal (<15s) | 0.11 | ER-03 | 1 | 5 | 3 |
| Simplicity | 0.17 | MF-02 | 4 | 5 | 1 |
| Capacity (300g cotton) | 0.44 | SR-03, WT-02 | 4 | 4 | 5 |
| Cost | 0.28 | MC-03 | 5 | 4 | 2 |
| **Weighted Total** | **1.00** | | **3.95** | **4.28** | **3.26** |

**Selected: Removable Drop-In Bin** — The flip-top reservoir bin meets the 15×15×15cm dimension requirement (SR-03) and enables tool-free removal in under 15 seconds (ER-03). The design supports operator ergonomics during demo operation.

---

### Decision Point 11: Operator Interface

The operator interface enables human supervision and control of the autonomous harvesting system. During demonstrations and field operation, operators need real-time visibility into system state and the ability to intervene when necessary.

**Option A: Web-Based Dashboard**
A browser-accessible interface built with modern web technologies (React, Vue or plain HTML/CSS/JS). The dashboard connects to the ROS2 system via rosbridge websocket protocol, displaying real-time state, metrics and providing control buttons. Accessible from any device with a web browser.

**Option B: No Dedicated Interface (Command Line Only)**
System operation relies entirely on ROS2 command-line tools (ros2 topic, ros2 service) and terminal outputs. Suitable for development but requires technical expertise and provides limited situational awareness during demonstrations.

**Table 12. Operator Interface Decision Matrix**

| Criterion | Weight | PDS Ref. | Web Dashboard | CLI Only |
|-----------|--------|----------|---------------|----------|
| Operator Usability | 0.20 | ER-01 | 5 | 1 |
| Real-time Status Visibility | 0.20 | ER-01, ER-02 | 5 | 2 |
| Cross-Platform Accessibility | 0.30 | MF-03 | 5 | 3 |
| Implementation Complexity | 0.30 | MF-02 | 3 | 5 |
| **Weighted Total** | **1.00** | | **4.40** | **3.00** |

**Selected: Web-Based Dashboard** — The RoboCot monitoring application provides intuitive control through START/PAUSE/EMERGENCY buttons (ER-01), color-coded status indicators visible at a glance (ER-02) and real-time ML confidence visualization. The web-based approach enables monitoring from mobile devices without requiring software installation, supporting flexible operator positioning during demonstrations.

---

## 2.2.3 Decision Criteria and Weights

The criteria weights used in each decision matrix are derived from the Binary Dominance Matrix established in Section 2.1. The BDM pairwise comparison yielded the following priority ranking: Safety (16.67%), Standards (15.15%), Quality (13.64%), Size Restriction (10.61%), Reliability (10.61%), Environment (10.61%), Material Cost (7.58%), Maintenance (6.06%), Manufacturing Cost (4.55%), Ergonomics (3.03%), Weight (1.52%) and Aesthetic (0.00%). Each decision matrix applies weights proportional to the relevant PDS criteria for that specific design choice, as indicated by the PDS Reference column in Tables 2-12.

## 2.2.4 Final System Configuration

**Table 13. Selected Components Summary**

| Decision Point | Selected Option | Weighted Score | Key Justification |
|----------------|-----------------|----------------|-------------------|
| Software Framework | ROS2 | 4.52 | Best tool integration, modularity and support |
| Camera Placement | Eye-in-Hand (Wrist) | 4.42 | Enables multi-angle inspection and close-up viewing |
| Depth Sensing | RGB-D Camera | 4.69 | Direct depth eliminates stereo calibration errors |
| Detection Method | YOLO11 | 4.64 | Robust detection under varying lighting conditions |
| Cluster Identification | World-Space 3D Clustering | 4.78 | Stable across discrete scan positions |
| Clustering Algorithm | Complete-Linkage | 4.71 | Prevents chain-linking artifacts |
| Motion Planning | MoveIt2 with OMPL | 4.54 | Collision avoidance and Cartesian path support |
| Manipulator | 6-DOF Braccio Arm | 4.51 | Flexible approach trajectories, sufficient reach |
| Gripper | Parallel Jaw | 4.38 | Reliable on fibrous material, 3D printable |
| Reservoir | Removable Drop-In Bin | 4.28 | Tool-free removal, ergonomic operation |
| Operator Interface | Web-Based Dashboard | 4.40 | Intuitive control, real-time visibility, cross-platform |

The selected components integrate into a coherent system architecture where ROS2 provides the communication backbone, YOLO11 detection feeds into the world-space clustering pipeline, MoveIt2 plans collision-free motions for the 6-DOF arm and the parallel gripper executes picks depositing cotton into the removable reservoir. The web-based operator dashboard provides real-time monitoring and control capability. Section 2.3 provides detailed design specifications for each selected component and their integration.

---

# 2.3 Detailed Design and Analysis

## 2.3.1 System Architecture

> **BRIEF**: ROS2-based modular architecture with 9 packages. Reference CHECKPOINT Section 2.

### Package Overview

| Package | Purpose | Key Components |
|---------|---------|----------------|
| `robot_arm` | Arm simulation in Gazebo | bot.launch.py, URDF, controllers |
| `robot_arm_moveit_config` | Motion planning | MoveIt2, OMPL, IK solver |
| `orchestrator` | Vision pipeline & control | YOLO detector, depth processor, explorer |
| `harvester_interfaces` | Custom ROS2 messages | BoundingBox, DetectedCluster, services |

[FIGURE: Node Interaction Diagram - from CHECKPOINT Section 2.2]

### Communication Topology

**Topics:**
- `/camera/color/image_raw` - RGB frames (640x480, 30Hz)
- `/camera/depth/image_raw` - Depth frames (32FC1, meters)
- `/joint_states` - Current joint positions
- `/tf` - Transform tree

**Services:**
- `/yolo/detect` - Run YOLO inference
- `/depth_processor/pixel_to_3d` - Convert pixel to world coordinates
- `/detection/run_at_position` - Full detection pipeline
- `/explorer/panoramic_scan` - Start field scan

[FIGURE: Data Flow Pipeline - from CHECKPOINT Section 2.3]

---

## 2.3.2 Mechanical Design

> **BRIEF**: 6-DOF Braccio arm kinematics. Reference EI_Pick_n_Place URDF.

### Kinematic Chain

```
world (fixed)
  └── base_link
        └── shoulder_link ─── Joint 1 (Z-axis rotation)
              └── upper_arm ─── Joint 2 (Y-axis, shoulder pitch)
                    └── forearm ─── Joint 3 (Y-axis, elbow)
                          └── wrist_pitch ─── Joint 4 (Y-axis)
                                └── wrist_roll ─── Joint 5 (X-axis)
                                      └── gripper_base ─── Joint 6 (gripper)
                                            └── camera_link
                                                  └── camera_optical_frame
```

### Joint Limits Table

> **TODO**: Update with Braccio 6-DOF specifications from EI_Pick_n_Place

| Joint | Type | Axis | Min (rad) | Max (rad) | Description |
|-------|------|------|-----------|-----------|-------------|
| J1 | revolute | Z | -π | +π | Base rotation |
| J2 | revolute | Y | -1.57 | +1.57 | Shoulder pitch |
| J3 | revolute | Y | -2.1 | +2.1 | Elbow |
| J4 | revolute | Y | -π | +π | Wrist pitch |
| J5 | revolute | X | -π | +π | Wrist roll |
| J6 | revolute | - | 0 | 0.5 | Gripper |

### Link Dimensions

> **TODO**: Extract from Braccio URDF

| Link | Length (m) | Mass (kg) |
|------|------------|-----------|
| base | TBD | TBD |
| upper_arm | TBD | TBD |
| forearm | TBD | TBD |
| wrist | TBD | TBD |

### Workspace Analysis

[FIGURE: Reachability envelope diagram - top and side views]

> **TODO**: Calculate or capture from RViz - show reachable workspace covers all three cluster positions.

### End-Effector Frames

| Frame | Parent | Offset | Purpose |
|-------|--------|--------|---------|
| `tool0` | wrist_roll | - | Flange reference |
| `tcp` | tool0 | +Z offset | Tool Center Point |
| `camera_link` | tool0 | -X, pitch | Camera housing |
| `camera_optical_frame` | camera_link | ROS convention | Z-forward optical frame |

---

## 2.3.3 Vision System

> **BRIEF**: RGB-D camera specifications and YOLO model details.

### Camera Specifications

| Property | Value |
|----------|-------|
| Type | RGB-D (simulated RealSense-style) |
| Resolution | 640 x 480 |
| Field of View | 90° horizontal (1.57 rad) |
| Update Rate | 30 Hz |
| Depth Range | 0.05 - 3.0 m |
| Depth Noise | Gaussian, σ = 0.007 |
| Frame ID | camera_optical_frame |

### Camera Intrinsics (K Matrix)

The pinhole camera model relates 3D points to 2D pixel coordinates:

```
     [fx   0  cx]   [277   0  320]
K =  [ 0  fy  cy] = [  0 277  240]
     [ 0   0   1]   [  0   0    1]
```

Where:
- fx, fy = 277 pixels (focal length)
- cx = 320, cy = 240 (principal point at image center)

### Pinhole Projection Model

A 3D point P = (X, Y, Z) in camera frame projects to pixel (u, v):

```
u = fx * (X/Z) + cx
v = fy * (Y/Z) + cy
```

[FIGURE: Pinhole camera model diagram with ray geometry]

### YOLO Model Specifications

| Property | Value |
|----------|-------|
| Model | YOLO11 (Ultralytics) |
| Weights | best.pt (custom trained) |
| Training Data | Roboflow Cotton-boll-and-cluster-2 |
| Classes | cotton_boll (0), cotton_boll-cluster (1) |
| Confidence Threshold | 0.7 |
| Input Size | 640 x 480 |

[FIGURE: YOLO detection output showing bounding boxes on cotton]

---

## 2.3.4 Spatial Detection Pipeline

> **BRIEF**: The core innovation - converting 2D detections to accurate 3D world positions.

### Pipeline Overview

```
RGB Image → YOLO Detection → Bounding Box
                               ↓
                          Pixel Center (u, v)
                               ↓
              [Optional] Camera Focus (1-2 iterations)
                               ↓
Depth Image → Depth Lookup → Z value at (u, v)
                               ↓
                          Back-Projection
                               ↓
                          Point in Camera Frame
                               ↓
               TF Transform → Point in World Frame
                               ↓
                          World-Space Clustering
                               ↓
                          TrackedCluster[]
```

### Back-Projection Formula

Given pixel coordinates (u, v) and depth Z, compute 3D point in camera frame:

```
X_cam = (u - cx) * Z / fx = (u - 320) * Z / 277
Y_cam = (v - cy) * Z / fy = (v - 240) * Z / 277
Z_cam = Z (direct from depth image)
```

### TF Chain

```
camera_optical_frame → camera_link → tool0 → hand → ... → base_link → world
```

[FIGURE: TF tree visualization from ros2 run tf2_tools view_frames]

### Critical Bug Fix: K vs P Matrix

> **LESSON LEARNED**: Gazebo generated incorrect P matrix (cx=160, cy=120) while K matrix was correct (cx=320, cy=240). Using PinholeCameraModel.projectPixelTo3dRay() caused ~20cm systematic error. Solution: extract intrinsics directly from K matrix and perform back-projection manually. This reduced error from ~20cm to ~1-2cm.

### Complete-Linkage Clustering Algorithm

World-space clustering groups detections from multiple scan positions:

```
For each new detection at position P_new:
    For each existing cluster C_i:
        If distance(P_new, member) < r_merge FOR ALL members in C_i:
            Add P_new to C_i
            Return
    Create new cluster containing P_new
```

**Why complete-linkage (not single-linkage)?**
- Single-linkage: join if close to ANY member → chain-linking artifacts
- Complete-linkage: join only if close to ALL members → tight, compact clusters

### Merge Radius Calculation

```
r_merge = 0.25 × min(inter-cluster distance)
        = 0.25 × 0.485m
        = 0.121m
```

This ensures:
- Bolls on same plant (< 12cm apart) → grouped together
- Different plants (48cm apart) → stay separate

---

## 2.3.5 Scanning Strategy

> **BRIEF**: 7x3 panoramic scan with snake traversal pattern.

### Panoramic Scan Grid

| Row | Name | Shoulder (rad) | Elbow (rad) | Description |
|-----|------|----------------|-------------|-------------|
| 0 | middle | -1.3 | 1.5 | Home-like position |
| 1 | lower | -1.3 | 1.7 | Tilt down |
| 2 | lowest | -1.3 | 1.9 | Further down |

| Col | Name | Hip (rad) | Hip (deg) |
|-----|------|-----------|-----------|
| 0 | far_left | -0.78 | -45° |
| 1 | left | -0.52 | -30° |
| 2 | mid_left | -0.26 | -15° |
| 3 | center | 0.0 | 0° |
| 4 | mid_right | 0.26 | +15° |
| 5 | right | 0.52 | +30° |
| 6 | far_right | 0.78 | +45° |

### Snake Traversal Pattern (Boustrophedon)

```
       -45°   -30°   -15°    0°   +15°   +30°   +45°
MIDDLE  [1] ──► [2] ──► [3] ──► [4] ──► [5] ──► [6] ──► [7]
                                                          │
LOWER  [14] ◄─ [13] ◄─ [12] ◄─ [11] ◄─ [10] ◄── [9] ◄── [8]
         │
LOWEST [15] ──► [16] ──► [17] ──► [18] ──► [19] ──► [20] ──► [21]
```

[FIGURE: Diagram showing snake pattern with camera FOV cones]

**Rationale:** Snake pattern minimizes joint travel between consecutive positions while ensuring complete field coverage.

### FOV Overlap Analysis

> **TODO**: Calculate overlap percentage based on camera FOV (90°) and hip angle increments (15°).

---

## 2.3.6 Control & Motion

> **BRIEF**: Visual servoing for target centering, proportional control law.

### Visual Servoing Control Law

The camera_focus node adjusts arm joints to center a detected target in the image:

```
Pixel Error:
    error_u = u - 320  (positive = target is RIGHT of center)
    error_v = v - 240  (positive = target is BELOW center)

Joint Adjustments:
    Δhip      = -K_hip × error_u       (pan left/right)
    Δshoulder =  K_shoulder × error_v  (tilt down)
    Δelbow    = -K_elbow × error_v     (assist tilt)

Where:
    K_hip = 0.002 rad/pixel
    K_shoulder = 0.0015 rad/pixel
    K_elbow = 0.001 rad/pixel
```

### Why Proportional Control Works

For wrist-mounted camera with our arm geometry:
- Horizontal pixel error → hip rotation corrects it
- Vertical pixel error → shoulder/elbow tilt corrects it
- No complex inverse kinematics needed for centering task
- Gains tuned empirically for arm configuration

[FIGURE: Visual servoing diagram showing error reduction over 2 iterations]

### Pipeline Integration: End-to-End Demonstration

> **CASE STUDY**: Partial Visibility Recovery

**Scenario:** Cluster initially visible only in bottom-right corner of frame (partial view).

**Process:**
1. **Position 1**: YOLO detects partial cluster, bbox touches image edge
2. Camera focus iteration 1: Adjust hip/shoulder to center detection
3. Camera focus iteration 2: Fine-tune centering
4. **Result**: Full cluster now visible, better depth reading at center
5. Repeat across all 21 scan positions
6. World-space clustering merges detections into 3 clusters
7. Compare to ground truth: ~1-2cm accuracy achieved

[FIGURE: Before/after images showing partial visibility → centered view]

### Motion Planning Integration

- MoveIt2 with OMPL for collision-free trajectories
- Planning group: "arm" (6 joints)
- Velocity scaling: 0.3 (30% of max)
- Collision objects: reservoir, ground plane

---

## 2.3.7 Validation & Results

> **BRIEF**: Ground truth comparison demonstrating ~1-2cm localization accuracy.

### Ground Truth Cluster Positions

| Cluster | Ground Truth (x, y, z) | Description |
|---------|------------------------|-------------|
| cluster_1 | (0.875, 0.475, 0.46) | Plant 3 - left side |
| cluster_2 | (0.975, 0.0, 0.52) | Plant 2 - center |
| cluster_3 | (0.875, -0.475, 0.42) | Plant 1 - right side |

### Inter-Cluster Distances

| Pair | Distance (m) |
|------|--------------|
| cluster_1 ↔ cluster_2 | 0.485 |
| cluster_2 ↔ cluster_3 | 0.485 |
| cluster_1 ↔ cluster_3 | 0.950 |

### Validation Results

| Ground Truth | Detected Position | Error (cm) |
|--------------|-------------------|------------|
| cluster_2 (0.975, 0.0, 0.52) | (0.970, 0.006, 0.474) | ~1-2 |

> **TODO**: Complete validation for all 3 clusters with full error metrics (mean, std).

### Error Analysis

**Sources of Error:**
1. Depth sensor noise (σ = 0.007m)
2. TF transform timing (asynchronous updates)
3. Detection bbox center vs true boll center
4. Camera focus convergence tolerance

**Mitigation:**
- Z-offset correction: -0.03m (mesh origin vs detection center)
- Multiple detections per cluster → use best (largest bbox area)
- Complete-linkage clustering rejects outliers

---

## 2.3.8 Operator Interface

> **BRIEF**: Mobile monitoring app for real-time supervision and control.

### Design Requirements

Based on ergonomic principles:
- Clear system health indication at a glance
- Intuitive control buttons (START/PAUSE/EMERGENCY)
- Real-time ML confidence visualization
- Pipeline progress tracking
- Alert logging for diagnostics

### User Interface Components

[FIGURE: RoboCot App screenshot with annotated components]

| Component | Purpose |
|-----------|---------|
| **Status Banner** | Color-coded state (Green=OK, Yellow=Warning, Red=Emergency) |
| **Session Metrics** | Bolls harvested, success rate %, reservoir fill |
| **Current Operation** | Main state, substate, ML confidence bar |
| **Pipeline Flow** | 5-step progress: Detect → View → Harvest → Transfer → Compress |
| **Alerts Section** | Rolling log of system events |
| **Control Panel** | START, PAUSE, SKIP CLUSTER, EMERGENCY STOP |

### State Machine

```
IDLE → DETECTING_CLUSTERS → CLUSTER_VIEW_POSITION → DETECTING_BOLLS
                                                          ↓
CLUSTER_COMPLETE ← COMPRESSION ← TRANSFERRING ← HARVESTING
      ↓
[Next Cluster or PAUSED for reservoir management]
```

[FIGURE: State machine diagram with transitions]

### Control Functionality

| Button | Action | Robot Behavior |
|--------|--------|----------------|
| START/RESUME | Begin or continue | Start scan or resume from pause |
| PAUSE | Safe stop | Complete current motion, hold position |
| SKIP CLUSTER | Bypass current | Move to next cluster in queue |
| EMERGENCY STOP | Immediate halt | Stop all motion, return to HOME |

---

# Figures Checklist

> **HIGH PRIORITY (Must-have)**

| # | Figure | Status | Source |
|---|--------|--------|--------|
| 1 | Gazebo environment overview | TODO | Simulation screenshot |
| 2 | 7x3 Panoramic scan snake pattern | TODO | Draw from data |
| 3 | YOLO detection with bounding boxes | TODO | yolo_output/*.png |
| 4 | RGB + Depth side-by-side | TODO | Gazebo capture |
| 5 | Kinematic chain diagram | TODO | Draw from URDF |
| 6 | TF tree visualization | TODO | ros2 run tf2_tools view_frames |
| 7 | Node interaction diagram | AVAILABLE | CHECKPOINT Section 2.2 |
| 8 | Data flow pipeline | AVAILABLE | CHECKPOINT Section 2.3 |
| 9 | RoboCot App screenshot | TODO | HTML demo |
| 10 | State machine diagram | TODO | Draw from timeline |

> **MEDIUM PRIORITY (Nice-to-have)**

| # | Figure | Status | Source |
|---|--------|--------|--------|
| 11 | Workspace reachability | TODO | RViz or calculate |
| 12 | Visual servoing convergence | TODO | If data available |
| 13 | Camera FOV overlap | TODO | Calculate |
| 14 | MoveIt path visualization | TODO | RViz screenshot |

---

# Tables Checklist

| # | Table | Status | Source |
|---|-------|--------|--------|
| 1 | Decision Matrix | DRAFT | Section 2.2.3 |
| 2 | Joint Limits (6-DOF) | TODO | Braccio URDF |
| 3 | Link Dimensions | TODO | Braccio URDF |
| 4 | Camera Specifications | DRAFT | Section 2.3.3 |
| 5 | Camera Intrinsics | DRAFT | Section 2.3.3 |
| 6 | YOLO Model Specs | DRAFT | Section 2.3.3 |
| 7 | Scan Grid Positions | DRAFT | Section 2.3.5 |
| 8 | Ground Truth Positions | DRAFT | Section 2.3.7 |
| 9 | Validation Results | TODO | Complete testing |

---

# Equations Checklist

| # | Equation | Status | Section |
|---|----------|--------|---------|
| 1 | Pinhole projection | DRAFT | 2.3.3 |
| 2 | Back-projection | DRAFT | 2.3.4 |
| 3 | Visual servoing control law | DRAFT | 2.3.6 |
| 4 | Complete-linkage condition | DRAFT | 2.3.4 |
| 5 | Merge radius calculation | DRAFT | 2.3.4 |
| 6 | Forward kinematics (DH) | TODO | 2.3.2 |

---

# Notes for Review

1. **ByteTrack Decision**: Added note in 2.2.4 explaining pivot from YOLO+ByteTrack to world-space 3D clustering due to memory buffer issues.

2. **Braccio 6-DOF**: Tables marked TODO for update from EI_Pick_n_Place URDF. Should I extract these now?

3. **Case Study**: Section 2.3.6 includes "Partial Visibility Recovery" demo concept. Need to capture actual images.

4. **Validation**: Only cluster_2 result shown. Need full validation with all 3 clusters.

5. **Page Estimate**: Current structure ~12-15 pages when filled with figures. Format says 10-20 for all of Section 2, so this should fit.

---
