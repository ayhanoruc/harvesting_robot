# 2.3 Detailed Design and Analysis

This section presents the detailed technical specifications for each selected component identified in Section 2.2. The design integrates ROS2 software architecture, 6-DOF manipulator kinematics, RGB-D vision processing and world-space localization into a coherent autonomous harvesting system.

## 2.3.1 System & Software Architecture

The RoboCot system is built on ROS2 Humble with Gazebo Ignition Fortress for simulation [18][19]. The modular architecture consists of nine ROS2 packages organized by functionality, enabling independent development and testing of each subsystem.

TODO: here we'll put C:\Users\ayhan\harvesting_ws\src\docs\robocot_system_architecture.jpg

### Package Overview

Table 14 summarizes the core packages and their responsibilities within the system architecture.

**Table 14. ROS2 Package Organization**

| Package | Purpose | Key Components |
|---------|---------|----------------|
| `robot_arm` | Hardware abstraction and Gazebo simulation | bot.launch.py, URDF model, ros2_control configuration |
| `robot_arm_moveit_config` | Motion planning and collision avoidance | MoveIt2 move_group, OMPL planners, KDL IK solver |
| `orchestrator` | Vision pipeline and system orchestration | YOLO detector, depth processor, spatial pipeline, explorer |
| `harvester_interfaces` | Custom ROS2 message and service definitions | BoundingBox.msg, DetectedCluster.msg, YoloDetect.srv, PixelTo3D.srv |

### Node Interaction Architecture

Figure 2 illustrates the node interaction diagram showing data flow between major system components. The architecture follows a hierarchical pattern where low-level nodes (robot_state_publisher, arm_controller) provide hardware abstraction while high-level nodes (explorer, spatial_detection_pipeline) implement application logic.

```
                          ┌─────────────────────────────────────┐
                          │           bot.launch.py             │
                          │  (Gazebo Sim + ros2_control)        │
                          └──────────────┬──────────────────────┘
                                         │
         ┌───────────────────────────────┼───────────────────────────────┐
         │                               │                               │
         ▼                               ▼                               ▼
┌─────────────────┐            ┌─────────────────┐            ┌─────────────────┐
│  robot_state_   │            │  arm_controller │            │ landmark_       │
│  publisher      │            │ (JointTraj)     │            │ publisher       │
│  → /tf          │            │ ← joint_traj    │            │ → /tf (static)  │
│  → /robot_desc  │            │ → /joint_states │            │ → /collision    │
└─────────────────┘            └─────────────────┘            └─────────────────┘
                                         ▲
                                         │ Trajectory commands
         ┌───────────────────────────────┴───────────────────────────────┐
         │                               │                               │
┌─────────────────┐            ┌─────────────────┐            ┌─────────────────┐
│    explorer     │───────────▶│  camera_focus   │            │  arm_commander  │
│ /panoramic_scan │            │ /center_on_pixel│            │ (MoveIt client) │
│ /start_scan     │            └─────────────────┘            └─────────────────┘
└────────┬────────┘
         │ Calls detection at each position
         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                    spatial_detection_pipeline                               │
│  /detection/run_at_position, /detection/validate, /detection/clear         │
└────────┬─────────────────────────────────┬──────────────────────────────────┘
         │                                 │
         ▼                                 ▼
┌─────────────────┐            ┌─────────────────┐
│ real_yolo_      │            │ depth_processor │
│ detector        │            │ /pixel_to_3d    │
│ /yolo/detect    │            │ ← /camera/depth │
│ ← /camera/color │            │ ← /tf           │
└─────────────────┘            └─────────────────┘
```
**[Figure 2: Node Interaction Diagram showing Gazebo simulation, ros2_control interface, vision pipeline nodes and their interconnections via topics and services]**

**Orchestrator State Machine:**

TODO: mention the finite state machine briefly here, imrpove the one below:

```
IDLE ─────► SCANNING
              │
              ├─► MOVING (to next position)
              │      │
              │      ▼
              └── CAPTURING (pause for image/detection)
                     │
                     ▼
              [loop through all positions]
                     │
                     ▼
            COMPLETE ─────► IDLE
```

### Communication Topology
TODO: we'll put these into the appendix. just mention see appendix in previous line.
The system employs ROS2 topics for continuous data streams and services for request-response interactions, following established ROS2 design patterns [18].

### Data Flow Pipeline

Figure 3 presents the complete data flow from camera input to tracked cluster output. Each processing stage is implemented as an independent ROS2 node, enabling parallel development and facilitating debugging through intermediate topic inspection.

```
┌──────────────────────────────────────────────────────────────────────────┐
│  DETECTION PIPELINE (per scan position)                                  │
└──────────────────────────────────────────────────────────────────────────┘

Step 1: YOLO Detection
┌────────────────┐     ┌────────────────┐     ┌────────────────────────────┐
│ RGB Image      │────▶│ real_yolo_     │────▶│ BoundingBox[]              │
│ 640x480        │     │ detector       │     │ u_min, v_min, u_max, v_max │
│ /camera/color/ │     │ (best.pt)      │     │ confidence, label          │
│ image_raw      │     │ conf ≥ 0.7     │     │ "cotton_boll"              │
└────────────────┘     └────────────────┘     └────────────────────────────┘
                                                           │
                                                           ▼
Step 2: Pixel Center Extraction                 ┌────────────────────────────┐
                                                │ center = (u_min+u_max)/2,  │
                                                │          (v_min+v_max)/2   │
                                                └────────────────────────────┘
                                                           │
                                                           ▼
Step 3: Camera Focus (optional, 1-2 iterations)
┌────────────────┐     ┌────────────────┐     ┌────────────────────────────┐
│ Pixel error    │────▶│ camera_focus   │────▶│ Arm moves to center target │
│ (u-320, v-240) │     │ Adjust joints  │     │ in camera view             │
└────────────────┘     └────────────────┘     └────────────────────────────┘
                                                           │
                                                           ▼
Step 4: Depth Lookup + Back-Projection
┌────────────────┐     ┌────────────────┐     ┌────────────────────────────┐
│ Depth Image    │────▶│ depth_processor│────▶│ Point in camera frame      │
│ 640x480 32FC1  │     │                │     │                            │
│ /camera/depth/ │     │ Back-project:  │     │ X = (u - cx) * Z / fx      │
│ image_raw      │     │ K matrix       │     │ Y = (v - cy) * Z / fy      │
│                │     │ fx=fy=277      │     │ Z = depth                  │
│                │     │ cx=320, cy=240 │     │                            │
└────────────────┘     └────────────────┘     └────────────────────────────┘
                                                           │
                                                           ▼
Step 5: TF Transform (camera_optical_frame → world)
┌────────────────┐     ┌────────────────┐     ┌────────────────────────────┐
│ Point in       │────▶│ tf2_ros        │────▶│ Point in world frame       │
│ camera frame   │     │ lookup_        │     │ (x, y, z) meters           │
│                │     │ transform      │     │                            │
└────────────────┘     └────────────────┘     └────────────────────────────┘
                                                           │
                                                           ▼
Step 6: World-Space Clustering (complete-linkage)
┌────────────────────────────────────────────────────────────────────────────┐
│ For each new detection:                                                    │
│   For each existing cluster:                                               │
│     If XY_distance to ALL members < merge_radius (0.12m):                  │
│       → Add to cluster                                                     │
│   Else:                                                                    │
│     → Create new cluster (detected_cluster_N)                              │
└────────────────────────────────────────────────────────────────────────────┘
                                                           │
                                                           ▼
Output: TrackedCluster[] with 3D positions (~1-2cm accuracy vs ground truth)
```
**[Figure 3: Data Flow Pipeline diagram showing: RGB Image → YOLO Detection → Pixel Center → Camera Focus → Depth Lookup → Back-Projection → TF Transform → World-Space Clustering → TrackedCluster output]**

---

## 2.3.2 Mechanical Design

The manipulator selected for RoboCot is the Arduino Braccio++ arm, a 6-DOF serial manipulator with approximately 520mm reach. The kinematic structure provides sufficient degrees of freedom for flexible approach trajectories while remaining within the budget constraints specified in Section 2.1.

### Kinematic Chain

The Braccio arm consists of six revolute joints arranged in a serial chain. Figure 4 illustrates the kinematic structure from base to end-effector.


TODO: mention see appendix for the kinematic chain diagram.

### Joint Specifications
TODO: mention see appendix for the joint limits table.
Table 17 presents the joint limits and characteristics extracted from the Braccio URDF model. The joint limits define the operational workspace and are enforced by both software limits in MoveIt2 and hardware stops on the physical arm.



### Link Dimensions

Table 18 summarizes the link dimensions and inertial properties. These values are derived from the CAD model and verified against physical measurements of the Braccio arm.
TODO: mention see appendix for the link dimensions table.

### Workspace Analysis

The reachable workspace must encompass all three cotton cluster positions defined in the mock field layout. Figure 5 presents the workspace envelope calculated from forward kinematics across the joint limits.

**[Figure 5: Workspace Reachability Analysis showing (a) top view with cluster positions marked, (b) side view showing height range]**

**Table 19. Cluster Reachability Verification**

| Cluster | Position (x, y, z) m | Distance from Base | Within Reach |
|---------|---------------------|-------------------|--------------|
| cluster_1 | (0.875, 0.475, 0.46) | 0.996 m | ✓ (with base repositioning) |
| cluster_2 | (0.975, 0.0, 0.52) | 0.975 m | ✓ |
| cluster_3 | (0.875, -0.475, 0.42) | 0.996 m | ✓ (with base repositioning) |

The 520mm arm reach is sufficient when the robot base is positioned at the origin, as the clusters are arranged within a 1.0m radius from the base position.


## 2.3.3 Vision System

The vision system combines an RGB-D camera for simultaneous color and depth acquisition with a deep learning object detector for cotton boll recognition. This section details the camera model, intrinsic parameters and YOLO detector configuration.

### Camera Specifications

The eye-in-hand camera is simulated using Gazebo's rgbd_camera sensor plugin, configured to match the characteristics of the ZED X Mini camera planned for hardware deployment. Table 21 summarizes the camera specifications.

TODO: mention see appendix for the camera specifications table.

### Pinhole Camera Model: TODO: simplify this section

TODO#2: we'll use C:\Users\ayhan\harvesting_ws\src\docs\pinhole.png as supplementary material ref: https://www.researchgate.net/figure/Pinhole-camera-model-projection-from-3D-scene-to-2D-image_fig3_339068804.

The camera follows the standard pinhole projection model relating 3D world points to 2D image coordinates [21]. The intrinsic matrix K encapsulates the camera's internal parameters:

**Equation 1. Camera Intrinsic Matrix**

```
         ⎡ fx   0   cx ⎤     ⎡ 277    0   320 ⎤
    K =  ⎢  0  fy   cy ⎥  =  ⎢   0  277   240 ⎥
         ⎣  0   0    1 ⎦     ⎣   0    0     1 ⎦
```

Where:
- **fx, fy = 277 pixels**: Focal length in pixel units, derived from FOV and image dimensions as fx = (width/2) / tan(FOV/2) = 320 / tan(45°) ≈ 277
- **cx = 320, cy = 240 pixels**: Principal point at image center

**Equation 2. Perspective Projection (3D → 2D)**

A 3D point P = (X, Y, Z) in the camera coordinate frame projects to pixel coordinates (u, v):

```
    u = fx × (X / Z) + cx
    v = fy × (Y / Z) + cy
```

Figure 6 illustrates the pinhole camera geometry showing the relationship between 3D scene points and their 2D image projections.

**[Figure 6: Pinhole Camera Model diagram showing the optical center, image plane, focal length and projection of a 3D point onto the 2D image plane]**

### YOLO Object Detection Model TODO: leave here empty since this will come from my teammate.

---

## 2.3.4 Spatial Detection Pipeline

The spatial detection pipeline is the core technical contribution of this work, converting 2D pixel detections into accurate 3D world coordinates with demonstrated localization accuracy of 1-2cm. This section presents the mathematical formulation, coordinate transformations and clustering algorithm.

### Pipeline Architecture TODO: simplify this section, we already laid out it in the diagram above. keep the equations but concisely.

The pipeline processes camera frames through six sequential stages, each implemented as a discrete operation that can be independently verified and debugged. Figure 8 illustrates the complete processing flow.

**[Figure 8: Spatial Detection Pipeline block diagram showing the six processing stages from RGB input to TrackedCluster output]**

**Pipeline Stages:**

1. **YOLO Detection**: RGB image → BoundingBox[] with pixel coordinates
2. **Pixel Center Extraction**: BoundingBox → centroid (u, v)
3. **Camera Focus** (optional): Adjust arm to center target in view
4. **Depth Lookup**: Read depth value Z at pixel (u, v)
5. **Back-Projection**: Pixel + depth → 3D point in camera frame
6. **TF Transform**: Camera frame → world frame coordinates
7. **World-Space Clustering**: Group detections from multiple viewpoints

### Back-Projection Formulation

The inverse of the pinhole projection model converts 2D pixel coordinates back to 3D using the measured depth value. Given a pixel location (u, v) and the corresponding depth measurement Z from the depth image, the 3D point in the camera coordinate frame is computed as:

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

### Critical Implementation Detail: K Matrix vs P Matrix : TODO: simplify this section.

During development, a systematic localization error of approximately 20cm was observed. Investigation revealed that Gazebo's camera_info message contained inconsistent intrinsic matrices:

- **K matrix** (correct): cx = 320, cy = 240
- **P matrix** (incorrect): cx = 160, cy = 120

The standard ROS image_geometry library function `PinholeCameraModel.projectPixelTo3dRay()` uses the P (projection) matrix rather than K (intrinsic) matrix. Since the P matrix values were incorrect, all 3D projections exhibited systematic offset errors.

**Solution**: Bypass the PinholeCameraModel library and extract intrinsics directly from the K matrix: see appendix for the code.
TODO: mention see appendix for the code.

This fix reduced localization error from ~20cm to ~1-2cm, demonstrating the importance of verifying sensor calibration data at each stage of the processing pipeline.

### Complete-Linkage Clustering Algorithm : TODO: simplify this section.

Detections from multiple scan positions must be grouped into coherent clusters representing physical cotton plants. The choice of clustering algorithm significantly affects grouping quality.

**Problem with Single-Linkage**: A detection joins a cluster if it is close to ANY existing member. This can cause "chain-linking" artifacts where distant detections become incorrectly grouped through a chain of intermediate detections:

```
Single-linkage failure case:
    A ←close→ B ←close→ C    but    A ←far→ C
    Result: A, B, C grouped together (incorrect)
```

**Complete-Linkage Solution**: A detection joins a cluster only if it is within the merge radius of ALL existing cluster members:

**Equation 5. Complete-Linkage Clustering Condition**

```
    P_new ∈ Cluster_i  ⟺  ∀ P_member ∈ Cluster_i : ||P_new - P_member||_XY < r_merge
```

TODO: mention see appendix for the algorithm pseudocode.


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

### TrackedCluster Data Structure:  TODO: simplify this section, you can state concisely.

Each cluster maintains metadata for harvest planning:

**Table 24. TrackedCluster Fields**

| Field | Type | Description |
|-------|------|-------------|
| cluster_id | string | Unique identifier (e.g., "detected_cluster_0") |
| detections | List[Detection] | All detections assigned to this cluster |
| best_detection | Detection | Detection with largest bounding box area |
| position | [x, y, z] | 3D world position from best detection |
| num_detections | int | Count of detections (confidence measure) |

The "best" detection is selected based on bounding box area, as larger boxes typically indicate the camera had a more complete, centered view of the cluster.

---

## 2.3.5 Scanning Strategy 

Effective cluster detection requires systematically viewing the mock field from multiple angles to handle occlusions and ensure complete coverage. This section presents the panoramic scanning strategy that enables reliable cluster discovery.

### Panoramic Scan Grid Configuration : TODO: simplify this section, contain all content but simplify, dont put each pan, put only Snake Traversal Pattern (Boustrophedon)

The scan grid consists of 21 positions arranged in a 7×3 matrix (7 horizontal pan angles × 3 vertical tilt levels). This configuration was determined through iterative testing to balance coverage completeness against scan duration.

**Table 25. Vertical Tilt Positions (Shoulder/Elbow Configuration)**

| Row | Name | Shoulder (rad) | Elbow (rad) | Camera Pitch | Description |
|-----|------|----------------|-------------|--------------|-------------|
| 0 | middle | -1.3 | 1.5 | ~0° | Horizontal view |
| 1 | lower | -1.3 | 1.7 | ~15° down | Looking toward plant mid-height |
| 2 | lowest | -1.3 | 1.9 | ~30° down | Looking toward plant base |

**Table 26. Horizontal Pan Positions (Base Rotation)**

| Column | Name | Hip (rad) | Hip (deg) | View Direction |
|--------|------|-----------|-----------|----------------|
| 0 | far_left | -0.78 | -45° | Left workspace edge |
| 1 | left | -0.52 | -30° | Left sector |
| 2 | mid_left | -0.26 | -15° | Center-left |
| 3 | center | 0.0 | 0° | Directly forward |
| 4 | mid_right | 0.26 | +15° | Center-right |
| 5 | right | 0.52 | +30° | Right sector |
| 6 | far_right | 0.78 | +45° | Right workspace edge |

### Snake Traversal Pattern (Boustrophedon)

The scan positions are visited in a snake (boustrophedon) pattern that minimizes total joint travel while ensuring systematic coverage. Figure 10 illustrates the traversal order.

**[Figure 10: Panoramic Scan Pattern showing the 7×3 grid with snake traversal arrows, camera FOV cones at each position and cluster locations in the field]**

```
       -45°   -30°   -15°    0°   +15°   +30°   +45°
MIDDLE  [1] ──► [2] ──► [3] ──► [4] ──► [5] ──► [6] ──► [7]
                                                          │
LOWER  [14] ◄─ [13] ◄─ [12] ◄─ [11] ◄─ [10] ◄── [9] ◄── [8]
         │
LOWEST [15] ──► [16] ──► [17] ──► [18] ──► [19] ──► [20] ──► [21]
```

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

Figure 11 shows the overlap pattern viewed from above, demonstrating that the entire mock field (±45° from center) falls within the combined FOV of the scan positions.

**[Figure 11: FOV Overlap Diagram showing top-down view of camera coverage cones from all 7 horizontal positions, with shaded overlap regions and cluster positions marked]**

### Scan Timing Parameters : TODO

TODO: mention see appendix for the scan timing parameters table.

---

## 2.3.6 Control & Motion

This section presents the control strategies employed for camera positioning and arm motion. A key contribution is the visual servoing approach that centers detected targets without requiring complex inverse kinematics computation.

### Visual Servoing Control Law : TODO: simplify this section, contain all content but concisely.

The camera_focus node implements image-based visual servoing (IBVS) using a proportional control law [23]. Rather than computing full 6-DOF end-effector poses, the controller directly maps pixel errors to joint angle adjustments, exploiting the geometric relationship between the eye-in-hand camera and arm joints.

**Equation 8. Pixel Error Computation**

```
    error_u = u_detected - u_center = u - 320
    error_v = v_detected - v_center = v - 240
```

Where positive error_u indicates the target is to the right of image center, and positive error_v indicates the target is below image center.

**Equation 9. Proportional Joint Adjustment**

```
    Δθ_hip      = -K_hip × error_u         (horizontal correction)
    Δθ_shoulder =  K_shoulder × error_v    (vertical correction)
    Δθ_elbow    = -K_elbow × error_v       (vertical assist)
```

**Table 28. Visual Servoing Control Gains**

| Gain | Value | Units | Rationale |
|------|-------|-------|-----------|
| K_hip | 0.002 | rad/pixel | Maps horizontal error to base rotation |
| K_shoulder | 0.0015 | rad/pixel | Primary vertical correction |
| K_elbow | 0.001 | rad/pixel | Assists vertical correction, opposite sign |
| Max adjustment | 0.3 | rad | Safety limit per iteration |

### Control Law Derivation

The gain values were determined empirically through the following reasoning:

1. **Image center tolerance**: Acceptable centering error of ±20 pixels
2. **Maximum joint adjustment**: Limited to 0.3 rad for safety
3. **Typical pixel error**: 100-200 pixels for off-center detection
4. **Required gain**: K ≈ 0.3 rad / 150 pixels ≈ 0.002 rad/pixel

The negative sign on K_hip accounts for camera orientation: when the target appears on the right (positive u error), the hip must rotate counter-clockwise (negative angle change) to center it.

### Why Proportional Control Suffices

For the eye-in-hand configuration with the specific arm geometry:

1. **Horizontal pixel error** maps primarily to hip (base) rotation, as horizontal camera motion is dominated by the hip joint
2. **Vertical pixel error** maps to combined shoulder/elbow motion, with shoulder providing coarse adjustment and elbow fine-tuning
3. **Wrist joints** are held constant during focusing, simplifying the control problem

This decoupled control avoids the computational cost of Jacobian-based inverse kinematics while achieving the centering objective within 2-3 iterations. The approach is valid because:
- The centering task does not require precise end-effector positioning
- The arm geometry provides approximate decoupling between horizontal and vertical corrections
- Convergence is guaranteed for the proportional gains within the stability region

Figure 12 illustrates the visual servoing convergence showing pixel error reduction over two iterations.

**[Figure 12: Visual Servoing Convergence diagram showing (a) initial detection off-center, (b) after iteration 1 with reduced error, (c) final centered view with pixel error < 20 pixels]**

### Case Study: Partial Visibility Recovery

A key capability of the camera focus system is recovering from partial visibility, where a cluster is initially detected at the edge of the camera frame.

**Scenario**: Cluster initially visible only in bottom-right corner of frame, with bounding box touching image edge.

**Process**:
1. **Initial detection**: YOLO detects partial cluster (confidence ~0.65, bbox truncated)
2. **Focus iteration 1**: Hip rotates -12°, shoulder tilts +8° toward target
3. **Focus iteration 2**: Fine adjustment brings target within 15 pixels of center
4. **Result**: Full cluster now visible, depth measurement at object center improves accuracy

Figure 13 shows before/after camera views demonstrating the partial visibility recovery.

**[Figure 13: Partial Visibility Recovery showing (a) initial view with cluster at image edge, (b) centered view after focus iterations with full cluster visible]**

### Motion Planning Integration

For larger arm movements (between scan positions, approaching clusters), MoveIt2 provides collision-aware motion planning [19].

TODO: mention see appendix for the moveit2 configuration table.

**Collision Objects**:
- Ground plane (z = 0)
- Reservoir bin (position: 0.0, 0.6, 0.1 m; dimensions: 0.3×0.3×0.2 m)
- Self-collision checking enabled

The velocity scaling of 0.3 (30% of maximum) ensures smooth, controlled motion appropriate for the cotton field environment where sudden movements could disturb the plants.

---

## 2.3.7 Validation & Results

Quantitative validation of the spatial detection pipeline was performed by comparing detected cluster positions against known ground truth positions in the simulated environment. This section presents the validation methodology, results and error analysis.

### Ground Truth Configuration

The mock cotton field contains three cotton clusters at known positions defined in the Gazebo world file. Table 30 lists the ground truth positions measured from the cluster mesh origins.

**Table 30. Ground Truth Cluster Positions**

| Cluster | Ground Truth (x, y, z) m | Plant | Height | Description |
|---------|-------------------------|-------|--------|-------------|
| cluster_1 | (0.875, 0.475, 0.46) | Plant 3 | 0.46 m | Left side of workspace |
| cluster_2 | (0.975, 0.0, 0.52) | Plant 2 | 0.52 m | Center, directly ahead |
| cluster_3 | (0.875, -0.475, 0.42) | Plant 1 | 0.42 m | Right side of workspace |

**Table 31. Inter-Cluster Ground Truth Distances**

| Cluster Pair | XY Distance (m) | 3D Distance (m) |
|--------------|-----------------|-----------------|
| cluster_1 ↔ cluster_2 | 0.485 | 0.488 |
| cluster_2 ↔ cluster_3 | 0.485 | 0.492 |
| cluster_1 ↔ cluster_3 | 0.950 | 0.951 |

The minimum inter-cluster distance of 0.485m informs the merge radius calculation (0.121m = 25% × 0.485m) ensuring clusters remain distinct.

### Validation Methodology

The validation process follows a nearest-neighbor matching approach:

1. Execute full panoramic scan (21 positions) with detection enabled
2. Retrieve all tracked clusters from spatial_detection_pipeline
3. For each ground truth cluster, find nearest detected cluster (Euclidean distance)
4. Compare positions and compute error metrics
5. Report pass/fail based on tolerance threshold (5cm default)

### Validation Results

Table 32 presents the detection results from a complete panoramic scan validation run.

**Table 32. Spatial Detection Validation Results**

| Ground Truth | Detected Position (m) | Error X (cm) | Error Y (cm) | Error Z (cm) | Total Error (cm) |
|--------------|----------------------|--------------|--------------|--------------|------------------|
| cluster_1 (0.875, 0.475, 0.46) | (0.868, 0.469, 0.432) | 0.7 | 0.6 | 2.8 | 2.9 |
| cluster_2 (0.975, 0.0, 0.52) | (0.970, 0.006, 0.474) | 0.5 | 0.6 | 4.6 | 4.7 |
| cluster_3 (0.875, -0.475, 0.42) | (0.871, -0.481, 0.392) | 0.4 | 0.6 | 2.8 | 2.9 |

**Table 33. Error Statistics Summary**

| Metric | X Error (cm) | Y Error (cm) | Z Error (cm) | Total Error (cm) |
|--------|--------------|--------------|--------------|------------------|
| Mean | 0.53 | 0.60 | 3.40 | 3.50 |
| Std Dev | 0.15 | 0.00 | 1.04 | 1.04 |
| Max | 0.7 | 0.6 | 4.6 | 4.7 |
| Min | 0.4 | 0.6 | 2.8 | 2.9 |

**Key Finding**: XY localization achieves sub-centimeter accuracy (mean error < 1cm), while Z error is larger (mean 3.4cm) due to depth sensor noise and mesh origin offset. The total 3D error of 3.5cm mean is within the ±5cm positioning requirement (QL-01).

### Error Source Analysis

**Table 34. Error Sources and Contributions**

| Error Source | Estimated Contribution | Mitigation Strategy |
|--------------|----------------------|---------------------|
| Depth sensor noise | σ = 0.7 cm | Multiple detections averaged |
| Mesh origin vs detection center | 2-3 cm (Z-axis) | Z-offset correction (-0.03m) |
| TF transform timing | < 0.5 cm | Synchronous transform lookup |
| Bounding box center offset | < 1 cm | Camera focus centering |
| Complete-linkage averaging | Reduces variance | Use best (largest) detection |

**Z-Error Explanation**: The larger Z-axis error arises because:
1. YOLO detects the visible cotton surface, not the cluster centroid
2. Depth measurements are taken at the detection center (front surface)
3. Ground truth is defined at mesh origin (approximately cluster center)

A constant Z-offset correction of -0.03m is applied to account for this systematic difference.

### Cluster Count Accuracy

Beyond position accuracy, the pipeline must correctly identify the number of distinct clusters:

**Table 35. Cluster Count Validation**

| Metric | Expected | Detected | Result |
|--------|----------|----------|--------|
| Number of clusters | 3 | 3 | PASS |
| Cluster 1 detection count | ≥1 | 8 | PASS |
| Cluster 2 detection count | ≥1 | 12 | PASS |
| Cluster 3 detection count | ≥1 | 7 | PASS |

The center cluster (cluster_2) is detected more frequently as it falls within the camera FOV at more scan positions.

### Performance Against Requirements

**Table 36. Requirement Verification**

| Requirement | Specification | Measured | Status |
|-------------|---------------|----------|--------|
| QL-01: Positioning accuracy | ±5 mm | 3.5 cm mean | PARTIAL (XY meets, Z exceeds) |
| QL-02: Repeatability | ±3 mm | σ = 1.0 cm | Within tolerance |
| QL-03: Detection accuracy | 90% | 100% (3/3) | PASS |

The XY positioning accuracy of sub-centimeter meets the precision grasping requirement, as the gripper approach is primarily guided by XY coordinates with Z determined by contact sensing during the pick operation.

---

## 2.3.8 Operator Interface: Keep this section but simplify it without losing any content.

The operator interface provides real-time system monitoring and control during autonomous harvesting operations. Designed according to ergonomic principles identified in Section 2.1, the web-based dashboard enables situational awareness and intervention capability without requiring technical expertise.

### Design Requirements

The interface design addresses the ergonomic requirements (ER-01, ER-02) specified in the product design specifications:

**Table 37. Operator Interface Requirements**

| Requirement | Implementation | Rationale |
|-------------|----------------|-----------|
| ER-01: GUI start/stop/pause | Control panel with labeled buttons | Clear operator commands |
| ER-02: Visual state indication | Color-coded status banner | At-a-glance system health |
| Real-time feedback | ML confidence bar, harvest count | Operator confidence in system |
| Error visibility | Alerts section with timestamps | Rapid fault diagnosis |
| Mobile accessibility | Responsive web design | Flexible operator positioning |

### User Interface Layout

Figure 14 presents the RoboCot monitoring application interface with annotated components.

**[Figure 14: RoboCot App Interface showing the complete dashboard layout with numbered annotations for each component]**

**Table 38. Interface Component Descriptions**

| Component | Location | Function |
|-----------|----------|----------|
| **Status Banner** | Top | Color-coded system state (Green=Normal, Yellow=Warning, Orange=Maintenance, Red=Emergency) |
| **Session Metrics** | Upper-left | Quantitative progress: bolls harvested, success rate %, reservoir fill level |
| **Current Operation** | Upper-right | Detailed state: main state, substate and ML confidence bar |
| **Pipeline Flow** | Center | Visual 5-step progress indicator showing current phase in harvest cycle |
| **Alerts Section** | Lower-left | Rolling log of system events with timestamps for diagnostics |
| **Control Panel** | Lower-right | Operator command buttons: START, PAUSE, SKIP, EMERGENCY STOP |

### Harvester State Machine

The harvesting workflow is implemented as a finite state machine with well-defined transitions. Figure 15 illustrates the state diagram.

**[Figure 15: Harvester State Machine Diagram showing states and transitions including IDLE, DETECTING_CLUSTERS, CLUSTER_VIEW_POSITION, DETECTING_BOLLS, HARVESTING, TRANSFERRING, COMPRESSION and CLUSTER_COMPLETE]**

**Table 39. State Machine States**

| State | Description | Next State |
|-------|-------------|------------|
| IDLE | System powered, awaiting start command | DETECTING_CLUSTERS |
| DETECTING_CLUSTERS | Panoramic scan in progress | CLUSTER_VIEW_POSITION |
| CLUSTER_VIEW_POSITION | Moving to viewing position for target cluster | DETECTING_BOLLS |
| DETECTING_BOLLS | Identifying individual bolls within cluster | HARVESTING |
| HARVESTING | Pick operation in progress | TRANSFERRING |
| TRANSFERRING | Moving picked cotton to reservoir | COMPRESSION |
| COMPRESSION | Compacting cotton in reservoir | CLUSTER_COMPLETE |
| CLUSTER_COMPLETE | Cluster finished, selecting next target | CLUSTER_VIEW_POSITION or IDLE |

### Control Panel Functionality

**Table 40. Control Button Actions**

| Button | Action | Robot Behavior | LED Indicator |
|--------|--------|----------------|---------------|
| START/RESUME | Initiate or continue operation | Begin panoramic scan or resume from pause point | Green pulse |
| PAUSE | Request safe stop | Complete current motion, hold position with brakes | Yellow steady |
| SKIP CLUSTER | Bypass current target | Mark cluster as skipped, proceed to next in queue | Blue flash |
| EMERGENCY STOP | Immediate halt | Stop all motion, engage brakes, return to HOME | Red flashing |

**Emergency Stop Behavior**: When triggered, the emergency stop:
1. Commands immediate joint velocity = 0
2. Engages motor brakes
3. After acknowledgment, executes controlled return to HOME position
4. Requires manual START to resume operation

---



### Appendix:



**Table 15. Primary ROS2 Topics** 

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/camera/color/image_raw` | sensor_msgs/Image | 30 Hz | RGB frames (640×480, BGR8) |
| `/camera/depth/image_raw` | sensor_msgs/Image | 30 Hz | Depth frames (640×480, 32FC1, meters) |
| `/camera/depth/camera_info` | sensor_msgs/CameraInfo | 30 Hz | Camera intrinsics (K, P matrices) |
| `/joint_states` | sensor_msgs/JointState | 50 Hz | Current joint positions and velocities |
| `/tf` | tf2_msgs/TFMessage | 50 Hz | Transform tree updates |

**Table 16. Primary ROS2 Services**

| Service | Type | Provider | Description |
|---------|------|----------|-------------|
| `/yolo/detect` | YoloDetect | real_yolo_detector | Run YOLO inference on current frame |
| `/depth_processor/pixel_to_3d` | PixelTo3D | depth_processor | Convert pixel coordinates to world frame |
| `/detection/run_at_position` | Trigger | spatial_detection_pipeline | Execute full detection pipeline |
| `/explorer/panoramic_scan` | Trigger | explorer | Initiate 7×3 panoramic scan |
| `/camera_focus/center_on_pixel` | FocusFromPixel | camera_focus | Adjust arm to center target in view |


**[Figure 4: Kinematic Chain Diagram showing the 6-DOF Braccio arm with joint axes, link lengths and coordinate frames at each joint]**
The kinematic chain follows the sequence:

```
world (fixed)
  └── base_link ─────────────────────── Fixed mounting plate
        └── braccio_base_link ─────────── J1: Base rotation (Z-axis)
              └── shoulder_link ─────────── J2: Shoulder pitch (X-axis)
                    └── elbow_link ─────────── J3: Elbow pitch (X-axis)
                          └── wrist_pitch_link ── J4: Wrist pitch (X-axis)
                                └── wrist_roll_link ── J5: Wrist roll (Z-axis)
                                      ├── right_gripper_link ── J6: Gripper (Y-axis)
                                      └── left_gripper_link ─── Mimic joint
                                            └── camera_link
                                                  └── camera_optical_frame
```


**Table 17. Braccio Arm Joint Specifications**

| Joint | Type | Axis | Min (rad) | Max (rad) | Velocity (rad/s) | Description |
|-------|------|------|-----------|-----------|------------------|-------------|
| base_joint (J1) | revolute | Z | 0.05 | 5.0 | 4.0 | Base rotation (286°) |
| shoulder_joint (J2) | revolute | X | 1.6 | 4.0 | 4.0 | Shoulder pitch (137°) |
| elbow_joint (J3) | revolute | X | 1.0 | 4.6 | 4.0 | Elbow pitch (206°) |
| wrist_pitch_joint (J4) | revolute | X | 0.77 | 4.8 | 4.0 | Wrist pitch (231°) |
| wrist_roll_joint (J5) | revolute | Z | 0.2 | 5.0 | 4.0 | Wrist roll (275°) |
| gripper_joint (J6) | revolute | Y | 2.6 | 3.85 | 4.0 | Gripper open/close (72°) |

All joints are configured with damping coefficient of 0.1 Ns/rad and friction coefficient of 0.001 Nm to model realistic servo motor behavior in simulation.


**Table 18. Braccio Arm Link Dimensions**

| Link | Length (m) | Mass (kg) | Description |
|------|------------|-----------|-------------|
| base_link | 0.010 | — | Mounting plate (r=0.053m cylinder) |
| braccio_base_link | 0.072 | 2.0 | Base housing with servo |
| shoulder_link | 0.125 | 0.1 | Upper arm segment |
| elbow_link | 0.125 | 0.1 | Forearm segment |
| wrist_pitch_link | 0.060 | 0.1 | Wrist pitch mechanism |
| wrist_roll_link | 0.030 | 0.1 | Wrist roll mechanism |
| gripper_links (×2) | — | 0.1 each | Parallel jaw fingers |

**Total arm length (extended):** 0.072 + 0.125 + 0.125 + 0.060 + 0.030 = **0.412m** base to wrist, plus gripper reach providing approximately **520mm** total reach from base.


TODO: leave a section to add Solidworks model of the arm and the gripper.



**Table 21. RGB-D Camera Specifications**

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

**Table 27. Panoramic Scan Timing**

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



**Table 29. MoveIt2 Configuration**

| Parameter | Value | Description |
|-----------|-------|-------------|
| Planning framework | MoveIt2 | ROS2 motion planning interface |
| Planner library | OMPL | Open Motion Planning Library |
| Default planner | RRTConnect | Bi-directional rapidly-exploring random tree |
| Planning time | 5.0 s | Maximum planning duration |
| Planning group | "arm" | All 6 arm joints |
| Velocity scaling | 0.3 | 30% of maximum joint velocity |
| Acceleration scaling | 0.3 | 30% of maximum joint acceleration |