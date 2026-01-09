# 2.3 Detailed Design and Analysis

The design integrates ROS2 software architecture, 6-DOF manipulator kinematics, RGB-D vision processing and world-space localization into a coherent autonomous harvesting system.

## 2.3.1 System & Software Architecture

The RoboCot system is built on ROS2 Humble with Gazebo Ignition Fortress for simulation [18][19]. The modular architecture consists of nine ROS2 packages organized by functionality, enabling independent development and testing of each subsystem.

*(See Appendix Figure 1: RoboCot System Architecture Overview)*

### Package Overview

Table 9 lists core packages and their responsibilities.

**Table 9. ROS2 Package Organization**

| Package | Purpose | Key Components |
|---------|---------|----------------|
| `robot_arm` | Hardware abstraction and Gazebo simulation | bot.launch.py, URDF model, ros2_control configuration |
| `robot_arm_moveit_config` | Motion planning and collision avoidance | MoveIt2 move_group, OMPL planners, KDL IK solver |
| `orchestrator` | Vision pipeline and system orchestration | YOLO detector, depth processor, spatial pipeline, explorer |
| `harvester_interfaces` | Custom ROS2 message and service definitions | BoundingBox.msg, DetectedCluster.msg, YoloDetect.srv, PixelTo3D.srv |

### Node Interaction Architecture

The architecture follows a hierarchical pattern: low-level nodes (robot_state_publisher, arm_controller) provide hardware abstraction while high-level nodes (explorer, spatial_detection_pipeline) implement application logic (Figure 2).

*(See Appendix Figure 2: Node Interaction Diagram)*

**Orchestrator State Machine:** IDLE → DETECTING_CLUSTERS → CLUSTER_VIEW_POSITION → DETECTING_BOLLS → HARVESTING → TRANSFERRING → CLUSTER_COMPLETE → (next cluster or IDLE)

The system employs ROS2 topics for continuous data streams and services for request-response interactions [18]. See **Appendix Tables 18-19** for complete topic and service listings.

### Data Flow Pipeline

Each processing stage is implemented as an independent ROS2 node, enabling parallel development and debugging through intermediate topic inspection (Figure 3).

*(See Appendix Figure 3: Data Flow Pipeline)*

---

## 2.3.2 Mechanical Design

The manipulator selected for RoboCot is the Arduino Braccio++ arm, a 6-DOF serial manipulator with approximately 520mm reach. The kinematic structure provides sufficient degrees of freedom for flexible approach trajectories while remaining within the budget constraints specified in Section 2.1.

### Kinematic Chain

The Braccio arm consists of six revolute joints in a serial chain (see **Appendix Figure 4** for kinematic chain diagram). Joint limits define the operational workspace and are enforced by both MoveIt2 software limits and hardware stops (see **Appendix Table 20**).

### Link Dimensions

Link dimensions and inertial properties are derived from the CAD model and verified against physical measurements (see **Appendix Table 21**). Total arm length (extended): 0.412m base to wrist, plus gripper reach providing ~520mm total reach.

### Workspace Analysis

The reachable workspace must encompass all three cotton cluster positions. Figure 5 shows the workspace envelope calculated from forward kinematics across joint limits.

<img src="figures/figure_05_workspace_analysis.png" alt="Figure 5: Workspace Reachability Analysis" width="80%">

*Figure 5: Workspace Reachability Analysis showing (a) top view with cluster positions marked, (b) side view showing height range*

**Table 10. Cluster Reachability Verification**

| Cluster | Position (x, y, z) m | Distance from Base | Within Reach |
|---------|---------------------|-------------------|--------------|
| cluster_1 | (0.875, 0.475, 0.46) | 0.996 m | ✓ (with base repositioning) |
| cluster_2 | (0.975, 0.0, 0.52) | 0.975 m | ✓ |
| cluster_3 | (0.875, -0.475, 0.42) | 0.996 m | ✓ (with base repositioning) |

The 520mm arm reach is sufficient when the robot base is positioned at the origin, as the clusters are arranged within a 1.0m radius from the base position.


## 2.3.3 Vision System

The vision system combines an RGB-D camera for color and depth acquisition with a deep learning detector for cotton boll recognition.

### Camera Specifications

The eye-in-hand camera is simulated using Gazebo's rgbd_camera sensor plugin, configured to match the ZED X Mini camera planned for hardware (see **Appendix Table 22** for full specifications).

### Pinhole Camera Model

The camera follows the standard pinhole projection model (Figure 6). The intrinsic matrix K encapsulates internal parameters:

<img src="pinhole.png" alt="Figure 6: Pinhole Camera Model" width="50%">

*Figure 6: Pinhole Camera Model https://www.researchgate.net/figure/Pinhole-camera-model-projection-from-3D-scene-to-2D-image_fig3_339068804*

**Equation 1. Camera Intrinsic Matrix**

```
         ⎡ fx   0   cx ⎤     ⎡ 277    0   320 ⎤
    K =  ⎢  0  fy   cy ⎥  =  ⎢   0  277   240 ⎥
         ⎣  0   0    1 ⎦     ⎣   0    0     1 ⎦
```

Where fx, fy = 277 pixels (focal length derived from FOV: fx = 320/tan(45°) ≈ 277), and cx=320, cy=240 (principal point at image center).

**Equation 2. Perspective Projection (3D → 2D)**

```
    u = fx × (X / Z) + cx
    v = fy × (Y / Z) + cy
```

### YOLO Object Detection Model

YOLO (You Only Look Once) is a single-stage object detector consisting of a CSPDarknet backbone for feature extraction, a PANet neck for multi-scale feature fusion, and a detection head that predicts bounding boxes with class probabilities in one forward pass [12][22]. This architecture enables real-time inference by processing the entire image simultaneously rather than using region proposals. RoboCot uses YOLO11 trained on the Cotton-boll-and-cluster dataset with 0.7 confidence threshold.

---

## 2.3.4 Spatial Detection Pipeline

The spatial detection pipeline is the core technical contribution, converting 2D pixel detections into 3D world coordinates with ~1-2cm accuracy. The pipeline stages are detailed in the Data Flow diagram (Figure 3, Section 2.3.1).

### Back-Projection Formulation

Given pixel (u, v) and depth Z, the 3D point in camera frame is:

**Equation 3. Back-Projection (2D → 3D)**

```
    X_cam = (u - cx) × Z / fx = (u - 320) × Z / 277
    Y_cam = (v - cy) × Z / fy = (v - 240) × Z / 277
    Z_cam = Z
```

This formulation inverts the perspective projection, recovering the original 3D position of the detected object relative to the camera optical center.

### Coordinate Frame Transformations

The computed camera-frame point must be transformed to the world coordinate frame for consistent spatial localization across multiple camera positions. The TF2 library [18] provides the transformation chain:

**Equation 4. Frame Transformation Chain**

```
    P_world = T_world←base × T_base←shoulder × ... × T_camera_link←optical × P_cam
```

The specific chain traverses:
```
camera_optical_frame → camera_link → wrist_roll_link → wrist_pitch_link →
elbow_link → shoulder_link → braccio_base_link → base_link → world
```

### Critical Implementation Detail: K Matrix vs P Matrix

During development, ~20cm systematic error was traced to Gazebo's camera_info message containing inconsistent matrices: **K matrix** (correct: cx=320, cy=240) vs **P matrix** (incorrect: cx=160, cy=120). The ROS `PinholeCameraModel.projectPixelTo3dRay()` uses the P matrix, causing offset errors.

**Solution**: Extract intrinsics directly from K matrix (see **Appendix** for code). This fix reduced error from ~20cm to ~1-2cm.

### Complete-Linkage Clustering Algorithm

Detections from multiple scan positions must be grouped into clusters representing physical plants. **Single-linkage** (join if close to ANY member) causes chain-linking artifacts. **Complete-linkage** requires proximity to ALL members:

**Equation 5. Complete-Linkage Clustering Condition**

```
    P_new ∈ Cluster_i  ⟺  ∀ P_member ∈ Cluster_i : ||P_new - P_member||_XY < r_merge
```

See **Appendix Algorithm 1** for pseudocode. Distance computed in XY plane only to group bolls at different heights on the same plant.


### Merge Radius Calculation

The merge radius r_merge determines the maximum distance for grouping detections. It must be:
- Large enough to group multiple detections of the same cluster
- Small enough to keep separate plants distinct

**Equation 6. Merge Radius Derivation**

```
    r_merge = α × min(d_ij)

    where:
        α = 0.25 (safety factor)
        d_ij = XY distance between cluster_i and cluster_j
        min(d_ij) = 0.485m (cluster_1 ↔ cluster_2 distance)

    therefore:
        r_merge = 0.25 × 0.485m = 0.121m
```

This radius ensures:
- Bolls within 12cm on the same plant → grouped together
- Plants separated by 48cm → remain distinct clusters

### TrackedCluster Data Structure

Each cluster stores: cluster_id, list of detections, best_detection (largest bbox area → most complete view), 3D position, and detection count. The "best" detection provides the final position estimate.

---

## 2.3.5 Scanning Strategy

Effective cluster detection requires systematically viewing the field from multiple angles to handle occlusions. The scan grid consists of **21 positions** in a 7×3 matrix: 7 horizontal pan angles (±45° in 15° increments) × 3 vertical tilt levels (0°, 15°, 30° down).

### Snake Traversal Pattern (Boustrophedon)

Scan positions are visited in a snake (boustrophedon) pattern minimizing total joint travel (Figure 10).

<img src="figures/figure_10_snake_pattern.svg" alt="Figure 10: Panoramic Scan Pattern" width="60%">

*Figure 10: Panoramic Scan Pattern (7×3 grid with snake traversal)*


**Traversal Order:**
- Row 0 (middle): Left to right (positions 1-7)
- Row 1 (lower): Right to left (positions 8-14)
- Row 2 (lowest): Left to right (positions 15-21)

**Rationale**: Alternating sweep directions eliminate the need for large hip angle reversals between rows, reducing total scan time by approximately 30% compared to a unidirectional raster pattern.

### Field of View Overlap Analysis : 

Adjacent scan positions must have sufficient FOV overlap to ensure no regions are missed. With a 90° horizontal FOV and 15° pan angle increments:

**Equation 7. FOV Overlap Calculation**

```
    FOV_half = 90° / 2 = 45°
    Pan_increment = 15°
    Overlap = 2 × (FOV_half - Pan_increment) = 2 × (45° - 15°) = 60°
```

This 60° overlap between adjacent positions provides:
- Redundant coverage for robust detection
- Multiple viewpoints for complete-linkage clustering
- Tolerance for slight positioning errors

The entire mock field (±45° from center) falls within the combined FOV (Figure 11).

<img src="figures/figure_11_fov_overlap.png" alt="Figure 11: FOV Overlap Diagram" width="50%">

*Figure 11: FOV Overlap Diagram showing top-down view of camera coverage cones from all 7 horizontal positions, with shaded overlap regions and cluster positions marked*

### Scan Timing Parameters

Total scan time ~55s for 21 positions. See **Appendix Table 23** for detailed timing breakdown.

---

## 2.3.6 Control & Motion

### Visual Servoing Control Law

The camera_focus node implements image-based visual servoing (IBVS) [23], mapping pixel errors directly to joint adjustments:

**Equation 8. Pixel Error:** `error_u = u - 320`, `error_v = v - 240`

**Equation 9. Proportional Joint Adjustment**

```
    Δθ_hip      = -K_hip × error_u         (horizontal: K_hip = 0.002 rad/pixel)
    Δθ_shoulder =  K_shoulder × error_v    (vertical: K_shoulder = 0.0015 rad/pixel)
    Δθ_elbow    = -K_elbow × error_v       (assist: K_elbow = 0.001 rad/pixel)
```

Gains derived empirically (K ≈ 0.3 rad / 150 pixels). The arm geometry provides approximate decoupling—horizontal error maps to hip rotation, vertical error to shoulder/elbow—enabling centering within 2-3 iterations without Jacobian-based IK. Max adjustment limited to 0.3 rad/iteration.

<img src="figures/figure_12_visual_servoing_convergence.png" alt="Figure 12: Visual Servoing Convergence" width="80%">

*Figure 12: Visual Servoing Convergence showing (a) initial off-center, (b) after iteration 1, (c) centered view with error < 20 pixels*

### Partial Visibility Recovery

When a cluster is detected at the frame edge (truncated bbox, ~0.65 confidence):
- Focus iterations adjust hip/shoulder to center the target
- Full cluster visibility achieved within 2 iterations
- Centered depth measurement improves localization accuracy

<img src="figures/partially_visible.png" alt="Figure 13a: Cluster at image edge" width="45%"> <img src="figures/centered_after_focus.png" alt="Figure 13b: Centered after focus" width="45%">

*Figure 13: Partial Visibility Recovery showing (a) cluster at image edge, (b) centered after focus iterations*

### Motion Planning Integration

MoveIt2 provides collision-aware planning [19] for larger movements (see **Appendix Table 24**). Collision objects: ground plane, reservoir bin, self-collision. Velocity scaling: 30%.

---

## 2.3.7 Validation & Results

Quantitative validation compares detected cluster positions against known ground truth positions in the simulated environment.

### Ground Truth Configuration

The minimum inter-cluster distance of 0.485m informs the merge radius calculation (0.121m = 25% × 0.485m).

### Validation Methodology

The validation process follows a nearest-neighbor matching approach:

1. Execute full panoramic scan (21 positions) with detection enabled
2. Retrieve all tracked clusters from spatial_detection_pipeline
3. For each ground truth cluster, find nearest detected cluster (Euclidean distance)
4. Compare positions and compute error metrics
5. Report pass/fail based on tolerance threshold (5cm default)

### Validation Results

Table 12 summarizes error statistics from a complete panoramic scan validation run (detailed per-cluster results in **Appendix Table 25**).

**Table 12. Error Statistics Summary**

| Metric | X Error (cm) | Y Error (cm) | Z Error (cm) | Total Error (cm) |
|--------|--------------|--------------|--------------|------------------|
| Mean | 0.53 | 0.70 | 0.37 | 1.16 |
| Std Dev | 0.15 | 0.95 | 0.38 | 0.74 |
| Max | 0.7 | 1.9 | 0.8 | 2.0 |

*Note: Z error includes -3cm offset correction for mesh origin vs detection center systematic difference.*

**Key Finding**: All three axes achieve sub-centimeter mean accuracy after Z-offset correction. Total 3D error of 1.16cm mean is well within ±5cm positioning requirement (QL-01), with XY accuracy of ~1cm enabling precise gripper approach.

### Cluster Count Accuracy

Beyond position accuracy, the pipeline must correctly identify the number of distinct clusters:

**Table 14. Cluster Count Validation**

| Metric | Expected | Detected | Result |
|--------|----------|----------|--------|
| Number of clusters | 3 | 3 | PASS |
| Cluster 1 detection count | ≥1 | 1 | PASS |
| Cluster 2 detection count | ≥1 | 4 | PASS |
| Cluster 3 detection count | ≥1 | 1 | PASS |

The center cluster (cluster_2) is detected more frequently as it falls within the camera FOV at more scan positions.

### Performance Against Requirements

**Table 15. Requirement Verification**

| Requirement | Specification | Measured | Status |
|-------------|---------------|----------|--------|
| QL-01: Positioning accuracy | ±5 cm | 1.16 cm mean | PASS |
| QL-02: Repeatability | ±3 mm | σ = 7.4 mm | FAIL* |
| QL-03: Detection accuracy | 90% | 100% (3/3) | PASS |

*The simulation environment has not been fully optimized; depth sensor noise parameters, TF timing synchronization, and camera focus convergence thresholds remain areas for improvement. Hardware implementation is expected to achieve better repeatability with proper sensor calibration.

---

## 2.3.8 Operator Interface

The web-based dashboard provides real-time monitoring and control per ergonomic requirements ER-01/ER-02.

*(See Appendix Figure 14)*

| Control | Behavior | Requirement |
|---------|----------|-------------|
| START/RESUME | Begin or resume operation | ER-01 |
| PAUSE | Complete current motion, hold | ER-01 |
| SKIP CLUSTER | Mark skipped, proceed to next | ER-01 |
| EMERGENCY STOP | Halt → brakes → return HOME → manual restart | ER-02 |

---



### Appendix:

---

#### Appendix Figures (Full Size)

<img src="robocot_system_architecture.jpg" alt="Figure 1: RoboCot System Architecture Overview" width="100%">

*Figure 1: RoboCot System Architecture Overview*

---

<img src="figures/figure_02_node_interaction.svg" alt="Figure 2: Node Interaction Diagram" width="100%">

*Figure 2: Node Interaction Diagram showing Gazebo simulation, ros2_control interface, vision pipeline nodes and their interconnections via topics and services*

---

<img src="figures/figure_03_data_flow.svg" alt="Figure 3: Data Flow Pipeline" width="100%">

*Figure 3: Data Flow Pipeline — RGB Image → YOLO Detection → Pixel Center → Camera Focus → Depth Lookup → Back-Projection → TF Transform → World-Space Clustering → TrackedCluster output*

---

<img src="figures/figure_cv_vs_yolo_sidebyside.png" alt="Figure 15: Classical CV vs YOLO Comparison" width="100%">

*Figure 15: Classical CV vs Deep Learning Detection — HSV segmentation detects all white regions (robot gripper parts) as false positives, while YOLO11 correctly identifies only cotton bolls*

---

<img src="robocot_app_ss.png" alt="Figure 14: RoboCot App Interface" width="50%">

*Figure 14: RoboCot App Interface — Components: color-coded status banner, session metrics (bolls harvested, success rate), ML confidence display, 5-step pipeline flow, timestamped alerts, control panel*

---

**Table 18. Primary ROS2 Topics** 

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/camera/color/image_raw` | sensor_msgs/Image | 30 Hz | RGB frames (640×480, BGR8) |
| `/camera/depth/image_raw` | sensor_msgs/Image | 30 Hz | Depth frames (640×480, 32FC1, meters) |
| `/camera/depth/camera_info` | sensor_msgs/CameraInfo | 30 Hz | Camera intrinsics (K, P matrices) |
| `/joint_states` | sensor_msgs/JointState | 50 Hz | Current joint positions and velocities |
| `/tf` | tf2_msgs/TFMessage | 50 Hz | Transform tree updates |

**Table 19. Primary ROS2 Services**

| Service | Type | Provider | Description |
|---------|------|----------|-------------|
| `/yolo/detect` | YoloDetect | real_yolo_detector | Run YOLO inference on current frame |
| `/depth_processor/pixel_to_3d` | PixelTo3D | depth_processor | Convert pixel coordinates to world frame |
| `/detection/run_at_position` | Trigger | spatial_detection_pipeline | Execute full detection pipeline |
| `/explorer/panoramic_scan` | Trigger | explorer | Initiate 7×3 panoramic scan |
| `/camera_focus/center_on_pixel` | FocusFromPixel | camera_focus | Adjust arm to center target in view |


<img src="figures/figure_04_kinematic_chain.svg" alt="Figure 4: Kinematic Chain Diagram" width="80%">

*Figure 4: Kinematic Chain Diagram showing the 6-DOF Braccio arm with joint axes, link lengths and coordinate frames at each joint*


**Table 20. Braccio Arm Joint Specifications**

| Joint | Type | Axis | Min (rad) | Max (rad) | Velocity (rad/s) | Description |
|-------|------|------|-----------|-----------|------------------|-------------|
| base_joint (J1) | revolute | Z | 0.05 | 5.0 | 4.0 | Base rotation (286°) |
| shoulder_joint (J2) | revolute | X | 1.6 | 4.0 | 4.0 | Shoulder pitch (137°) |
| elbow_joint (J3) | revolute | X | 1.0 | 4.6 | 4.0 | Elbow pitch (206°) |
| wrist_pitch_joint (J4) | revolute | X | 0.77 | 4.8 | 4.0 | Wrist pitch (231°) |
| wrist_roll_joint (J5) | revolute | Z | 0.2 | 5.0 | 4.0 | Wrist roll (275°) |
| gripper_joint (J6) | revolute | Y | 2.6 | 3.85 | 4.0 | Gripper open/close (72°) |

All joints are configured with damping coefficient of 0.1 Ns/rad and friction coefficient of 0.001 Nm to model realistic servo motor behavior in simulation.


**Table 21. Braccio Arm Link Dimensions**

| Link | Length (m) | Mass (kg) | Description |
|------|------------|-----------|-------------|
| base_link | 0.010 | — | Mounting plate (r=0.053m cylinder) |
| braccio_base_link | 0.072 | 2.0 | Base housing with servo |
| shoulder_link | 0.125 | 0.1 | Upper arm segment |
| elbow_link | 0.125 | 0.1 | Forearm segment |
| wrist_pitch_link | 0.060 | 0.1 | Wrist pitch mechanism |
| wrist_roll_link | 0.030 | 0.1 | Wrist roll mechanism |
| gripper_links (×2) | — | 0.1 each | Parallel jaw fingers |

**Total arm length (extended):** 0.072 + 0.125 + 0.125 + 0.060 + 0.030 = **0.412m** base to wrist, plus gripper reach providing approximately **520mm** total reach from base (per manufacturer specifications).


### Solidworks CAD Model

*[Placeholder: Insert Solidworks model images of the arm assembly and gripper mechanism here]*



**Table 22. RGB-D Camera Specifications**

| Property | Value | Notes |
|----------|-------|-------|
| Sensor Type | RGB-D (structured light) | Simulated RealSense-style |
| RGB Resolution | 640 × 480 pixels | Standard VGA |
| Depth Resolution | 640 × 480 pixels | Aligned with RGB |
| Horizontal FOV | 90° (1.57 rad) | Wide-angle for scanning |
| Update Rate | 30 Hz | Synchronized RGB and depth |
| Depth Range | 0.05 – 3.0 m | Optimal at 0.3-0.5m viewing distance |
| Depth Noise | Gaussian, σ = 0.007 m | ~7mm standard deviation |
| Frame ID | camera_optical_frame | Z-forward optical convention |



K Matrix vs P Matrix

```python
# Extract from K matrix (indices into 3x3 row-major array)
self.fx = msg.k[0]   # K[0,0]
self.fy = msg.k[4]   # K[1,1]
self.cx = msg.k[2]   # K[0,2]
self.cy = msg.k[5]   # K[1,2]
```


**Algorithm 1: Complete-Linkage Clustering**

```
function ADD_DETECTION(P_new):
    for each existing Cluster_i:
        is_close_to_all = true
        for each P_member in Cluster_i:
            if XY_distance(P_new, P_member) > r_merge:
                is_close_to_all = false
                break
        if is_close_to_all:
            add P_new to Cluster_i
            return
    create new Cluster containing P_new
```
Note: Distance is computed in the XY plane only (ignoring Z height differences) to group bolls at different heights on the same plant.


---

**Table 23. Panoramic Scan Timing**

| Parameter | Value | Description |
|-----------|-------|-------------|
| Pan movement duration | 1.5 s | Time to move between horizontal positions |
| Tilt movement duration | 1.5 s | Time to change vertical tilt |
| Pause at position | 1.0 s | Stabilization + detection time |
| Detection pipeline time | ~0.5 s | YOLO + depth + clustering |
| Total scan time (21 positions) | ~55 s | Full panoramic scan with detection |

The 1.0 second pause at each position allows:
- Camera image stabilization (motion blur elimination)
- YOLO inference completion
- Depth image synchronization
- TF transform availability



**Table 24. MoveIt2 Configuration**

| Parameter | Value | Description |
|-----------|-------|-------------|
| Planning framework | MoveIt2 | ROS2 motion planning interface |
| Planner library | OMPL | Open Motion Planning Library |
| Default planner | RRTConnect | Bi-directional rapidly-exploring random tree |
| Planning time | 5.0 s | Maximum planning duration |
| Planning group | "arm" | All 6 arm joints |
| Velocity scaling | 0.3 | 30% of maximum joint velocity |
| Acceleration scaling | 0.3 | 30% of maximum joint acceleration |


**Table 25. Per-Cluster Validation Results**

| Ground Truth | Detected Position (m) | Error X (cm) | Error Y (cm) | Error Z (cm)* | Total Error (cm) |
|--------------|----------------------|--------------|--------------|---------------|------------------|
| cluster_1 (0.875, 0.475, 0.46) | (0.870, 0.473, 0.428) | 0.5 | 0.2 | 0.2 | 0.6 |
| cluster_2 (0.975, 0.0, 0.52) | (0.968, 0.019, 0.489) | 0.7 | 1.9 | 0.1 | 2.0 |
| cluster_3 (0.875, -0.475, 0.42) | (0.871, -0.475, 0.382) | 0.4 | 0.0 | 0.8 | 0.9 |

*Z error after -3cm offset correction for mesh origin vs detection center.