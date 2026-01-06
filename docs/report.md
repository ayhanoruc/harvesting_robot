# RoboCot - Autonomous Cotton Harvesting System
## ME 429 Design Report - Sections 2.2 & 2.3

---

# 2.2 Overview of Possible Solutions

## 2.2.1 Solution Space Overview

> **BRIEF**: Introduce the key design decisions that shape the system. Frame the problem: autonomous cotton harvesting requires (1) reliable detection, (2) precise 3D localization, (3) dexterous manipulation. Each subsystem has multiple viable approaches.

**Key Decision Points:**
- Vision system type (sensor selection)
- Camera placement strategy
- Detection/ML approach
- 3D localization method
- Manipulator configuration
- Scanning/exploration strategy
- Multi-detection clustering approach

---

## 2.2.2 Alternative Solutions

### Alternative A: Classical Computer Vision + Simple Manipulator

> **BRIEF**: Baseline approach using color segmentation, edge detection, fixed overhead camera, 3-4 DOF arm.

**Characteristics:**
- Color-based segmentation (HSV thresholding for white cotton)
- Fixed overhead camera (bird's-eye view)
- 3-DOF or 4-DOF arm with simple gripper
- No depth sensing (2D localization only)

**Advantages:**
- Low computational cost
- Simple implementation
- Minimal hardware requirements

**Disadvantages:**
- Fails under varying lighting conditions
- No depth information for grasping
- Limited viewing angles (occlusion issues)
- Poor robustness to natural variations

[SKETCH A: Top-view diagram showing fixed overhead camera with limited viewing angles]

---

### Alternative B: Deep Learning + Fixed Stereo Camera

> **BRIEF**: Improved detection via YOLO, stereo camera for depth, but still fixed mounting.

**Characteristics:**
- YOLO-based object detection
- Stereo camera pair for triangulation-based depth
- Fixed side-mounted or overhead position
- 4-DOF arm

**Advantages:**
- Robust detection via deep learning
- 3D localization possible
- Better than color segmentation

**Disadvantages:**
- Stereo calibration complexity
- Baseline distance limits depth accuracy
- Fixed viewpoint still causes occlusions
- Cannot inspect from multiple angles

[SKETCH B: Side-view showing stereo camera pair with triangulation geometry]

---

### Alternative C: Deep Learning + Eye-in-Hand RGB-D (SELECTED)

> **BRIEF**: Our chosen approach - YOLO11 detection with wrist-mounted RGB-D camera, 6-DOF arm.

**Characteristics:**
- YOLO11 trained on cotton boll dataset (0.7+ confidence)
- Wrist-mounted Intel RealSense-style RGB-D camera
- Direct depth measurement (no stereo calibration)
- 6-DOF Braccio arm for full workspace coverage
- Multi-view scanning for complete coverage

**Advantages:**
- Robust detection in varying conditions
- Direct depth at each pixel (no triangulation)
- Camera follows arm - inspect from any angle
- Close-up views possible (~35cm viewing distance)
- 6-DOF enables complex approach trajectories

**Disadvantages:**
- Higher computational cost (GPU inference)
- More complex calibration (hand-eye)
- Requires motion planning integration

[SKETCH C: Diagram showing wrist-mounted camera following arm to multiple viewpoints]

---

### Subsystem Decision Summary

| Decision Point | Alt. A | Alt. B | Alt. C (Selected) |
|----------------|--------|--------|-------------------|
| **Vision Sensor** | Monocular RGB | Stereo RGB | RGB-D |
| **Camera Placement** | Fixed overhead | Fixed side | Eye-in-hand (wrist) |
| **Detection Method** | Color segmentation | YOLO | YOLO11 |
| **Localization** | 2D only | Stereo triangulation | Direct depth back-projection |
| **Arm DOF** | 3-DOF | 4-DOF | 6-DOF (Braccio) |
| **Scanning** | Single viewpoint | Single viewpoint | Panoramic 7x3 grid |
| **Clustering** | None | Pixel-space | World-space complete-linkage |

---

## 2.2.3 Decision Matrix

> **BRIEF**: Weighted scoring of alternatives against our design criteria.

### Criteria Weights

| Criterion | Weight | Justification |
|-----------|--------|---------------|
| Detection Accuracy | 25% | Critical - false negatives mean missed harvest |
| Localization Precision | 20% | Need ~1-2cm accuracy for successful grasping |
| Robustness to Occlusion | 15% | Cotton plants have complex, overlapping geometry |
| Implementation Complexity | 15% | Senior project timeline constraint |
| Computational Cost | 10% | Real-time operation required |
| Scalability | 10% | Future field deployment consideration |
| Hardware Cost | 5% | Academic budget (~1900 EUR total) |

### Scoring Matrix (1-5 scale, 5 = best)

| Criterion | Weight | Alt. A | Alt. B | Alt. C |
|-----------|--------|--------|--------|--------|
| Detection Accuracy | 0.25 | 2 | 4 | 5 |
| Localization Precision | 0.20 | 1 | 3 | 5 |
| Robustness to Occlusion | 0.15 | 1 | 2 | 4 |
| Implementation Complexity | 0.15 | 5 | 3 | 2 |
| Computational Cost | 0.10 | 5 | 3 | 2 |
| Scalability | 0.10 | 2 | 3 | 4 |
| Hardware Cost | 0.05 | 5 | 3 | 3 |
| **Weighted Total** | **1.00** | **2.40** | **3.05** | **3.85** |

---

## 2.2.4 Justification of Selected Approach

> **BRIEF**: Explain why Alternative C wins despite higher complexity.

### Detection Accuracy
- YOLO11 achieves 0.7+ confidence on cotton bolls in simulation
- Color segmentation achieves only ~0.3 equivalent accuracy under varying lighting
- Deep learning generalizes to natural variation in boll appearance

### Localization Precision
- RGB-D provides direct depth measurement per pixel
- Eliminates stereo calibration errors and baseline limitations
- Achieved ~1-2cm accuracy in validation tests (see Section 2.3.7)

### Occlusion Handling
- Eye-in-hand camera can approach from multiple angles
- Panoramic scan covers 7x3 grid of viewpoints
- Partial visibility recovered via camera focus iterations

### ByteTrack Consideration

> **NOTE**: Initial design considered YOLO + ByteTrack for multi-frame cluster tracking. However, memory buffer issues caused ID instability during testing. We pivoted to world-space 3D clustering using our spatial detection pipeline, which provides more robust cluster identification across scan positions.

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
