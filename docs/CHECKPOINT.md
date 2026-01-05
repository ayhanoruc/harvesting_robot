# Project Checkpoint - Robocot Cotton Harvesting System

**Date:** 2026-01-06
**Status:** Spatial Detection Pipeline Validated (~1-2cm accuracy)
**Platform:** ROS2 Humble + Gazebo Ignition Fortress + YOLO11

---

## 1. Project Overview

### What is Robocot?

Robocot is an autonomous cotton harvesting robot(6-DOF) on a fixed base, built on ROS2. Currently, for simplicity, it uses a 4-DOF robotic arm with a wrist-mounted RGB-D camera to detect, locate, and harvest cotton bolls from plants in a simulated field.

### Eventual Goal: Full Harvesting Cycle

```
HOME_VIEW (Panoramic Scan)
    │
    ├─ Sweep camera left-to-right (7×3 grid)
    ├─ Run YOLO at each position → detect cotton_boll class
    ├─ Convert pixel detections to 3D world coordinates
    ├─ Cluster nearby detections → identify cluster_1, cluster_2, cluster_3
    └─ Store 3D positions for each cluster
           │
           ▼
    ┌──────────────────────────────────────────────────┐
    │           HARVESTING LOOP (per cluster)          │
    └──────────────────────────────────────────────────┘
           │
           ▼
CLUSTER_VIEW (Approach + Full View)
    │
    ├─ Move to viewing position for cluster_N
    ├─ Camera focus iterations → center cluster in view
    ├─ YOLO detect individual bolls within cluster
    └─ Store boll positions (relative to cluster)
           │
           ▼
    ┌──────────────────────────────────────────────────┐
    │            BOLL PICKING LOOP (per boll)          │
    └──────────────────────────────────────────────────┘
           │
           ▼
BOLL_PICK (Visual Servoing + Grasp)
    │
    ├─ Focus on boll_M → center in view
    ├─ Approach: move TCP toward boll position
    ├─ Grasp: close gripper
    └─ Deposit: move to reservoir, release
           │
           ▼
    [Next boll in cluster, or next cluster]
           │
           ▼
    COMPLETE → Return to HOME
```

### Current Progress

- [x] 4-DOF arm simulation in Gazebo working
- [x] Wrist-mounted RGB-D camera with correct intrinsics
- [x] Real YOLO integration (cotton_boll detection, 0.7+ confidence)
- [x] Spatial detection pipeline (pixel → 3D world, ~1-2cm accuracy)
- [x] Panoramic scan (7×3 snake grid)
- [x] World-space clustering with complete-linkage algorithm
- [x] Ground truth validation framework
- [ ] **PENDING:** CLUSTER_VIEW positioning
- [ ] **PENDING:** Individual boll detection within cluster
- [ ] **PENDING:** Pick-and-place execution
- [ ] **PENDING:** Braccio 6-DOF arm integration
- [ ] **PENDING:** Full cycle demo video

---

## 2. System Architecture

### 2.1 Package Overview (9 packages)

| Package | Purpose | Key Nodes |
|---------|---------|-----------|
| `robot_arm` | 4-DOF arm + Gazebo simulation | bot.launch.py, landmark_publisher |
| `robot_arm_moveit_config` | MoveIt 2 motion planning | moveit.launch.py |
| `orchestrator` | Vision pipeline + system control | explorer, spatial_detection_pipeline, real_yolo_detector, depth_processor, camera_focus |
| `harvester_interfaces` | Custom messages/services | BoundingBox, DetectedCluster, YoloDetect, PixelTo3D |
| `vision_ml` | (Legacy) Mock detection | vision_ml_node |
| `robotic_actor` | (Legacy) Gripper simulation | main.py |
| `logger_node` | System logging | logger_node |
| `example_arm_description` | (Learning) 2-DOF example | display.launch.py |
| `example_arm_moveit_config` | (Learning) MoveIt example | demo.launch.py |

### 2.2 Node Interaction Diagram

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

### 2.3 Data Flow: Camera to 3D Position

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

---

## 3. Robot Arm (robot_arm package)

### 3.1 Kinematic Structure

**4-DOF Arm + 2-DOF Gripper** (6 controlled joints total)

```
world (fixed)
  └── base_link ─────────────────────── [fixed joint]
        │
        └── torso ───────────────────── hip (continuous, Z-axis)
              │                         Base rotation: ±π rad
              │
              └── upper_arm ─────────── shoulder (revolute, Y-axis)
                    │                   Tilt: -90° to +40° (-1.57 to 0.70 rad)
                    │
                    └── lower_arm ───── elbow (revolute, Y-axis)
                          │             Bend: ±120° (-2.1 to 2.1 rad)
                          │
                          └── hand ──── wrist (continuous, Y-axis)
                                │       Rotation: ±π rad
                                │
                                ├── left_gripper_base ── l_g_base (revolute, X-axis)
                                │     └── left_gripper_finger (fixed)
                                │
                                └── right_gripper_base ─ r_g_base (revolute, X-axis)
                                      └── right_gripper_finger (fixed)
```

**Joint Limits Table:**

| Joint | Type | Axis | Min | Max | Description |
|-------|------|------|-----|-----|-------------|
| `hip` | continuous | Z | -π | +π | Base rotation (pan) |
| `shoulder` | revolute | Y | -1.5708 (-90°) | 0.6981 (+40°) | Upper arm tilt |
| `elbow` | revolute | Y | -2.1 (-120°) | 2.1 (+120°) | Forearm bend |
| `wrist` | continuous | Y | -π | +π | Hand rotation |
| `l_g_base` | revolute | X | 0 | 0.5236 (+30°) | Left gripper open |
| `r_g_base` | revolute | X | -0.5236 (-30°) | 0 | Right gripper open |

**End-Effector Frames:**

| Frame | Parent | Offset | Purpose |
|-------|--------|--------|---------|
| `tool0` | hand | +0.025m Z | Flange reference |
| `tcp` | tool0 | +0.045m Z | Tool Center Point (red sphere) |
| `camera_link` | tool0 | -0.04m X, -90° pitch | Camera housing |
| `camera_optical_frame` | camera_link | -90° roll, -90° yaw | ROS optical convention (Z forward) |

### 3.2 URDF Summary

**File:** `robot_arm/urdf/mybot.urdf.xacro`

**Link Dimensions:**

| Link | Geometry | Dimensions | Mass |
|------|----------|------------|------|
| base_link | cylinder | r=0.1m, h=0.05m | 1.0 kg |
| torso | cylinder | r=0.05m, h=0.5m | 1.0 kg |
| upper_arm | cylinder | r=0.05m, h=0.4m | 1.0 kg |
| lower_arm | cylinder | r=0.05m, h=0.4m | 1.0 kg |
| hand | box | 0.05×0.05×0.05m | 1.0 kg |
| gripper_base (×2) | box | 0.005×0.005×0.02m | 0.01 kg |
| gripper_finger (×2) | box | 0.008×0.008×0.05m | 0.01 kg |
| camera_link | box | 0.02×0.03×0.02m | 0.05 kg |

**Camera Sensor (Gazebo):**

| Property | Value |
|----------|-------|
| Type | RGB-D (rgbd_camera) |
| Resolution | 640×480 |
| FOV | 90° horizontal (1.57 rad) |
| Update rate | 30 Hz |
| Depth range | 0.05 - 3.0 m |
| Intrinsics | fx=fy=277, cx=320, cy=240 |
| Noise | Gaussian, σ=0.007 |
| Frame ID | camera_optical_frame |

### 3.3 ros2_control Configuration

**File:** `robot_arm/yaml/controllers.yaml`

**Hardware Interface:** `ign_ros2_control/IgnitionSystem` (Gazebo plugin)

The `gz_ros2_control` plugin in the URDF creates the controller_manager inside Gazebo - no separate `ros2_control_node` needed.

**Controllers:**

| Controller | Type | Rate | Joints |
|------------|------|------|--------|
| `joint_state_broadcaster` | JointStateBroadcaster | 50 Hz | all 6 |
| `arm_controller` | JointTrajectoryController | 100 Hz | all 6 |

**arm_controller Configuration:**
- Command interface: `position`
- State interfaces: `position`, `velocity`
- `allow_partial_joints_goal: true` - can command subset of joints
- `open_loop_control: true` - no feedback loop in controller
- Topic: `/arm_controller/joint_trajectory`

**Launch Sequence** (`bot.launch.py`):

```
1. Gazebo Sim (cotton_field.world)
2. robot_state_publisher (URDF → /tf, /robot_description)
3. spawn_entity (robot into Gazebo)
4. ros_gz_bridge + ros_gz_image (topic bridges)
      ↓ [wait for spawn]
5. joint_state_broadcaster
      ↓ [wait for broadcaster]
6. arm_controller
      ↓ [wait for controller]
7. landmark_publisher (static TF + collision objects)
```

---

## 4. Vision Pipeline

### 4.1 Real YOLO Detector

**File:** `orchestrator/real_yolo_detector.py`

Runs actual YOLO11 inference on camera frames using the ultralytics library.

**Model:**
- Path: `orchestrator/models/best.pt`
- Classes: `cotton_boll` (0), `cotton_boll-cluster` (1)
- Trained on: Roboflow dataset (Cotton-boll-and-cluster-2)

**Two Service Endpoints:**

| Service | Purpose | Output |
|---------|---------|--------|
| `/yolo/detect` | Raw YOLO detections | All detected bounding boxes as-is |
| `/yolo/detect_clusters` | Pixel-space merged clusters | Nearby bolls grouped into cluster bboxes |

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | auto-detect | Path to YOLO .pt model |
| `confidence` | 0.7 | Detection confidence threshold |
| `camera_topic` | `/camera/color/image_raw` | RGB camera topic |
| `cluster_pixel_distance` | 150 | Max pixel distance for cluster merging |
| `save_images` | True | Save annotated detection images |
| `output_dir` | `yolo_output/` | Where to save images |

**Cluster Merging (pixel-space):**

The `/yolo/detect_clusters` endpoint uses complete-linkage clustering in pixel space:
- Only `cotton_boll` detections are grouped (not `cotton_boll-cluster`)
- A boll joins a cluster only if within `cluster_pixel_distance` of ALL existing members
- Returns merged bounding boxes with combined area and max confidence

Note: Pixel-space clustering is a quick heuristic. For reliable 3D localization, use the spatial_detection_pipeline which clusters in world-space (meters).

---

### 4.2 Spatial Detection Pipeline

**File:** `orchestrator/spatial_detection_pipeline.py`

Orchestrates the full detection → 3D localization → clustering pipeline.

**Pipeline Steps:**

```
1. YOLO Detection (/yolo/detect)
      │
      ▼ BoundingBox[] with pixel coordinates
2. Extract pixel centers: (u_min+u_max)/2, (v_min+v_max)/2
      │
      ▼
3. Camera Focus (optional, 1-2 iterations)
      │ Adjust arm to center target in view
      │ Re-detect to get updated bbox
      ▼
4. Depth Lookup (/depth_processor/pixel_to_3d)
      │
      ▼ Point in world frame (x, y, z)
5. Z-offset correction (-0.03m)
      │ Ground truth is mesh origin, detection is center
      ▼
6. World-Space Clustering (complete-linkage)
      │ Group by X,Y distance only (ignore Z)
      ▼
7. TrackedCluster[] with best detection per cluster
```

**Services:**

| Service | Type | Description |
|---------|------|-------------|
| `/detection/run_at_position` | Trigger | Run pipeline at current position |
| `/detection/validate` | Trigger | Compare detections to ground truth |
| `/detection/clear` | Trigger | Clear all tracked clusters |
| `/detection/print_results` | Trigger | Log all tracked clusters |
| `/detection/wait_ready` | Trigger | Wait for YOLO + depth services ready |

**Topics:**
- `/detection/status` (String) - Pipeline status updates
- `/detection/current_position` (String) - Current scan position name (from explorer)

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `focus_iterations` | 2 | Camera focus iterations per detection |
| `validation_tolerance` | 0.05 | Pass/fail threshold in meters (5cm) |
| `z_offset_correction` | 0.03 | Z adjustment for mesh origin (3cm) |
| `merge_radius` | 0.0 (auto) | Clustering radius in meters |
| `merge_radius_factor` | 0.25 | 25% of min cluster distance |
| `save_images` | True | Save annotated images with clusters |

**Complete-Linkage Clustering Algorithm:**

```
def add_detection(new_pos):
    for each existing_cluster:
        if XY_distance(new_pos, member) < merge_radius for ALL members:
            add to existing_cluster
            return
    create new_cluster(detected_cluster_N)
```

Why complete-linkage (not single-linkage):
- Single-linkage: boll joins if close to ANY member → chain-linking artifacts
- Complete-linkage: boll joins only if close to ALL members → tight, compact clusters

**Auto Merge Radius Calculation:**

```
merge_radius = min_XY_distance_between_clusters × 0.25
             = 0.485m × 0.25
             = 0.121m
```

This ensures bolls on the same plant group together while keeping separate plants apart.

**TrackedCluster Data Structure:**

| Field | Type | Description |
|-------|------|-------------|
| `cluster_id` | str | e.g., `detected_cluster_0` |
| `detections` | List[Detection] | All detections in this cluster |
| `best_detection` | Detection | Highest bbox_area (most complete view) |
| `position` | np.array | 3D position from best detection |
| `num_detections` | int | Count of detections |

**Detection Data Structure:**

| Field | Type | Description |
|-------|------|-------------|
| `cluster_label` | str | YOLO class label |
| `position_3d` | np.array | [x, y, z] in world frame |
| `confidence` | float | YOLO confidence |
| `bbox_area` | int | Bounding box area in pixels |
| `scan_position` | str | Scan position name |
| `pixel_center` | tuple | (u, v) pixel coordinates |

---

### 4.3 Depth Processor

**File:** `orchestrator/depth_processor.py`

Converts 2D pixel coordinates to 3D world coordinates using RGB-D camera data.

**Service:** `/depth_processor/pixel_to_3d` (PixelTo3D)

**Input:** Pixel coordinates (u, v)
**Output:** 3D point in world frame (x, y, z), success, message

**Back-Projection Formula:**

```
Camera Frame:
    X_cam = (u - cx) × depth / fx
    Y_cam = (v - cy) × depth / fy
    Z_cam = depth

Then: TF transform camera_optical_frame → world
```

**Camera Intrinsics (from K matrix):**

| Parameter | Value | Description |
|-----------|-------|-------------|
| fx | 277 | Focal length X |
| fy | 277 | Focal length Y |
| cx | 320 | Principal point X (image center) |
| cy | 240 | Principal point Y (image center) |

**Critical Bug Fix: K Matrix vs P Matrix**

**Problem:** Gazebo generated P matrix with wrong principal point (cx=160, cy=120 instead of 320, 240), causing ~20cm systematic error in all 3D projections.

**Root Cause:** `PinholeCameraModel.projectPixelTo3dRay()` uses the P matrix, not K matrix. Even after adding `<intrinsics>` to URDF, the P matrix remained incorrect.

**Solution:** Bypass PinholeCameraModel entirely. Extract intrinsics directly from K matrix in camera_info:

```python
# Store K matrix values directly (bypass buggy P matrix)
self.fx = msg.k[0]   # K[0,0]
self.fy = msg.k[4]   # K[1,1]
self.cx = msg.k[2]   # K[0,2]
self.cy = msg.k[5]   # K[1,2]

# Back-project using K matrix directly
point_cam.x = (u - self.cx) * depth / self.fx
point_cam.y = (v - self.cy) * depth / self.fy
point_cam.z = depth
```

**Result:** Error reduced from ~20cm to ~1-2cm

**Subscribed Topics:**
- `/camera/depth/camera_info` (CameraInfo) - Intrinsics
- `/camera/depth/image_raw` (Image) - Depth image (32FC1, meters)

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_info_topic` | `/camera/depth/camera_info` | Intrinsics topic |
| `depth_image_topic` | `/camera/depth/image_raw` | Depth topic |
| `camera_frame` | `camera_optical_frame` | Camera TF frame |
| `world_frame` | `world` | Target TF frame |
| `depth_scale` | 1.0 | Depth units (1.0 = meters) |

---

### 4.4 Camera Focus

**File:** `orchestrator/camera_focus.py`

Centers the camera on a target pixel by adjusting arm joint angles. Uses simple proportional control - no complex 3D geometry.

**Service:** `/camera_focus/center_on_pixel` (FocusFromPixel)

**Input:** Pixel coordinates (u, v)
**Output:** success, message

**Control Law (pixel-error heuristic):**

```
Pixel Error:
    error_u = u - 320  (positive = target is RIGHT)
    error_v = v - 240  (positive = target is DOWN)

Joint Adjustments:
    hip_delta     = -gain_hip × error_u      (pan left/right)
    shoulder_delta = gain_shoulder × error_v  (tilt down)
    elbow_delta   = -gain_elbow × error_v     (assist tilt)

New Joints:
    hip'      = hip + hip_delta
    shoulder' = shoulder + shoulder_delta
    elbow'    = elbow + elbow_delta
    wrist'    = wrist (unchanged)
```

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gain_hip` | 0.002 | rad per pixel (horizontal) |
| `gain_shoulder` | 0.0015 | rad per pixel (vertical) |
| `gain_elbow` | 0.001 | rad per pixel (vertical assist) |
| `image_center_u` | 320 | Image center X |
| `image_center_v` | 240 | Image center Y |
| `max_adjustment` | 0.3 | Max radians per call |

**Why Simple Heuristic Works:**

For wrist-mounted camera with arm geometry:
- Horizontal error → hip rotation (base pan)
- Vertical error → shoulder/elbow tilt

No IK solver needed. The gains are tuned empirically for the arm's kinematic configuration.

**Motion Execution:**
- Uses MoveGroup action client (MoveIt)
- Planning group: "arm"
- Velocity/acceleration scaling: 0.3
- Tolerance: ±0.01 rad

---

## 5. Exploration & Scanning

### 5.1 Panoramic Scan

**File:** `orchestrator/explorer.py` (method: `_execute_panoramic_scan_thread`)

Full field-of-view sweep using joint-space positions. Used for initial cluster discovery.

**Grid Configuration:** 7 columns × 3 rows = 21 positions

**Joint Angles:**

| Row | Name | Shoulder | Elbow | Description |
|-----|------|----------|-------|-------------|
| 0 | middle | -1.3 | 1.5 | Home-like position |
| 1 | lower | -1.3 | 1.7 | Camera tilts down |
| 2 | lowest | -1.3 | 1.9 | Camera tilts further down |

| Col | Name | Hip (rad) | Hip (deg) |
|-----|------|-----------|-----------|
| 0 | far_left | -0.78 | -45° |
| 1 | left | -0.52 | -30° |
| 2 | mid_left | -0.26 | -15° |
| 3 | center | 0.0 | 0° |
| 4 | mid_right | 0.26 | +15° |
| 5 | right | 0.52 | +30° |
| 6 | far_right | 0.78 | +45° |

**Snake Pattern (boustrophedon):**

Minimizes arm travel between positions:

```
       -45°   -30°   -15°    0°   +15°   +30°   +45°
MIDDLE  [1] ──► [2] ──► [3] ──► [4] ──► [5] ──► [6] ──► [7]
                                                          │
LOWER  [14] ◄─ [13] ◄─ [12] ◄─ [11] ◄─ [10] ◄── [9] ◄── [8]
         │
LOWEST [15] ──► [16] ──► [17] ──► [18] ──► [19] ──► [20] ──► [21]
```

Row 0: left → right
Row 1: right → left (reversed)
Row 2: left → right

**Workflow:**

```
1. Clear previous detections
2. Wait for detection pipeline ready
3. For each position in snake order:
   a. Move to joints (hip, shoulder, elbow, wrist)
   b. Pause for capture (1.0s default)
   c. Run detection pipeline (if enabled)
4. Return to HOME position
5. Validate against ground truth
```

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `pan_hip_angles` | [-0.78, ..., 0.78] | 7 hip angles |
| `pan_shoulder_range` | [-1.3, -1.3, -1.3] | 3 shoulder values |
| `pan_elbow_range` | [1.5, 1.7, 1.9] | 3 elbow values |
| `pan_pause_duration` | 1.0 | Seconds at each position |
| `pan_move_duration` | 1.5 | Seconds to move between positions |
| `enable_detection` | True | Run detection at each position |

---

### 5.2 Arc Sweep

**File:** `orchestrator/explorer.py` (method: `_execute_scan_thread`)

Cluster-focused scan with circular arc viewpoints around each known cluster. Used for detailed views after clusters are discovered.

**Arc Geometry:**

```
                    [CLUSTER]
                        *
                      / | \
                     /  |  \
                    v1 v2 v3 v4 v5  ← viewpoints on arc
                        |
                     [ROBOT]
```

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `view_distance` | 0.35 | Distance from cluster to viewpoint (m) |
| `arc_angle_deg` | 90 | Total arc span (±45° from center) |
| `views_per_cluster` | 5 | Number of viewpoints per cluster |
| `height_variation` | 0.06 | Z variation across arc (m) |
| `pause_at_viewpoint` | 1.5 | Seconds at each viewpoint |

**Viewpoint Calculation:**

```
For each cluster at (cx, cy, cz):
    base_angle = atan2(cy, cx)  # Direction from robot to cluster

    For each arc offset from -45° to +45°:
        view_angle = base_angle + π + arc_offset
        vx = cx + view_distance × cos(view_angle)
        vy = cy + view_distance × sin(view_angle)

        # Parabolic height profile (higher at edges)
        height_offset = height_variation × (arc_offset/45°)²
        vz = cz + 0.08 + height_offset
```

**Sequence:**

```
1. explore_start position (left side)
2. cluster_1: 5 viewpoints (arc)
3. cluster_2: 5 viewpoints (arc)
4. cluster_3: 5 viewpoints (arc)
5. explore_end position (right side)
```

---

### 5.3 Explorer Node

**File:** `orchestrator/explorer.py`

Orchestrates robot arm movement for scanning patterns.

**Services:**

| Service | Type | Description |
|---------|------|-------------|
| `/explorer/panoramic_scan` | Trigger | Start joint-space panoramic sweep |
| `/explorer/start_scan` | Trigger | Start arc sweep around known clusters |

**Topics Published:**

| Topic | Type | Description |
|-------|------|-------------|
| `/explorer/scan_status` | String | Current state (IDLE, SCANNING, MOVING, etc.) |
| `/explorer/viewpoint_reached` | String | Viewpoint info when reached |
| `/explorer/scan_position` | String | Position details during panoramic scan |
| `/detection/current_position` | String | Current position name for detection pipeline |

**State Machine:**

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

**Motion Execution:**

- **Panoramic scan:** Direct joint trajectory via `/arm_controller/joint_trajectory`
- **Arc sweep:** Via arm_commander service (`/go_to_pose` + `/arm_commander/set_parameters`)

**Threading:**

- Scans run in daemon threads to avoid blocking service callbacks
- Uses `threading.Lock` (`_scan_lock`) for thread-safe state management
- MultiThreadedExecutor with 4 threads

**Config Loading:**

Loads from `environment_config.yaml`:
- Cluster positions (for arc viewpoint calculation)
- Landmark positions (explore_start, explore_end)

---

## 6. Custom Interfaces (harvester_interfaces)

**Package:** `harvester_interfaces` (ament_cmake)

Custom ROS2 message and service definitions for the harvesting pipeline.

### 6.1 Messages

#### BoundingBox.msg

Represents a 2D detection from YOLO.

| Field | Type | Description |
|-------|------|-------------|
| `u_min` | int32 | Left pixel coordinate |
| `v_min` | int32 | Top pixel coordinate |
| `u_max` | int32 | Right pixel coordinate |
| `v_max` | int32 | Bottom pixel coordinate |
| `confidence` | float32 | Detection confidence [0-1] |
| `label` | string | Class label (e.g., "cotton_boll") |
| `area` | int32 | Box area in pixels (width × height) |

**Usage:** Returned by `/yolo/detect` service, passed to depth_processor for 3D conversion.

#### DetectedCluster.msg

Represents a tracked cotton cluster with 3D position (after spatial processing).

| Field | Type | Description |
|-------|------|-------------|
| `cluster_id` | string | Unique identifier (e.g., "detected_cluster_0") |
| `position` | geometry_msgs/Point | 3D position in world frame (x, y, z) |
| `confidence` | float32 | Best detection confidence |
| `best_bbox_area` | int32 | Area of best bounding box (largest = most complete view) |
| `num_detections` | int32 | How many times this cluster was detected across scan |
| `best_scan_position` | string | Scan position name where best detection occurred |

**Usage:** Output of spatial_detection_pipeline, used for harvesting loop planning.

### 6.2 Services

#### YoloDetect.srv

Run YOLO detection on current camera frame.

```
# Request: (empty)
---
# Response:
harvester_interfaces/BoundingBox[] detections
bool success
string message
```

**Providers:** `/yolo/detect`, `/yolo/detect_clusters` (real_yolo_detector)

#### PixelTo3D.srv

Convert 2D pixel coordinates to 3D world coordinates using depth + TF.

```
# Request:
int32 u    # Horizontal pixel coordinate (column)
int32 v    # Vertical pixel coordinate (row)
---
# Response:
geometry_msgs/Point position    # x, y, z in world frame
bool success
string message
```

**Provider:** `/depth_processor/pixel_to_3d` (depth_processor)

#### FocusFromPixel.srv

Center camera on a target pixel by adjusting arm joints.

```
# Request:
int32 u                    # Horizontal pixel coordinate
int32 v                    # Vertical pixel coordinate
float64 view_distance      # Distance from target (default: 0.35m if 0)
---
# Response:
geometry_msgs/Point tcp_position    # Where TCP should move to
float64 target_distance             # Distance from TCP to target
bool success
string message
```

**Provider:** `/camera_focus/center_on_pixel` (camera_focus)

#### FocusFromPosition.srv

Center camera on a 3D world position.

```
# Request:
geometry_msgs/Point target    # Target position in world frame
float64 view_distance         # Distance from target (default: 0.35m if 0)
---
# Response:
geometry_msgs/Point tcp_position    # Where TCP should move to
float64 target_distance             # Distance from TCP to target
bool success
string message
```

**Usage:** Alternative to FocusFromPixel when 3D position is already known.

#### RunDetectionPipeline.srv

Run the full spatial detection pipeline (detect → focus → 3D) at current position.

```
# Request:
int32 focus_iterations    # Number of focus iterations (default: 2)
---
# Response:
harvester_interfaces/DetectedCluster[] detections
bool success
string message
```

**Usage:** Called by explorer at each scan position.

---

## 7. Environment Configuration

**File:** `robot_arm/config/environment_config.yaml`

Single source of truth for all environment positions and parameters.

### 7.1 World Bounds & Robot Position

| Property | Value | Description |
|----------|-------|-------------|
| Frame ID | `world` | All positions in world frame |
| X bounds | -0.3 to 1.2 m | Reachable workspace |
| Y bounds | -0.8 to 0.8 m | Left-right extent |
| Z bounds | 0.0 to 1.0 m | Height range |
| Robot spawn | (0.0, 0.0, 0.1) | base_link position |

### 7.2 Ground Truth Cluster Positions

Cotton cluster positions in world frame (boll centers):

| Cluster | Position (x, y, z) | Stem Height | Description |
|---------|-------------------|-------------|-------------|
| `cluster_1` | (0.875, 0.475, 0.46) | 0.46 m | Plant 3 - left side |
| `cluster_2` | (0.975, 0.0, 0.52) | 0.52 m | Plant 2 - center |
| `cluster_3` | (0.875, -0.475, 0.42) | 0.42 m | Plant 1 - right side |

**Inter-cluster Distances (X,Y only):**

| Pair | Distance | Notes |
|------|----------|-------|
| cluster_1 ↔ cluster_2 | 0.485 m | Min distance (used for merge_radius) |
| cluster_2 ↔ cluster_3 | 0.485 m | Same distance |
| cluster_1 ↔ cluster_3 | 0.950 m | Diagonal |

### 7.3 Exploration Endpoints

Scan path endpoints for panoramic sweep:

| Landmark | Position (x, y, z) | Description |
|----------|-------------------|-------------|
| `explore_start` | (0.4, 0.45, 0.55) | Begin scan - left edge |
| `explore_end` | (0.4, -0.45, 0.55) | End scan - right edge |

**Exploration Parameters:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| Strategy | linear_sweep | Y+ to Y- sweep |
| Viewpoints | 5 | Scan positions (arc sweep) |
| Scan height | 0.55 m | Z height during scanning |
| Scan distance | 0.4 m | X distance from base |

### 7.4 Collision Objects

For MoveIt planning scene (avoid collisions):

| Object | Type | Position | Dimensions |
|--------|------|----------|------------|
| `reservoir` | box | (0.0, 0.6, 0.1) | 0.3×0.3×0.2 m |

**Reservoir:** Cotton drop zone at robot's left side.

### 7.5 Coordinate System Reference

```
                Y+
                ↑
                │    cluster_1
                │    (0.875, 0.475)
                │         ●
                │
    reservoir   │         
    (0.0, 0.6)  │    
                │         ● cluster_2
    ROBOT       |        (0.975, 0.0)
    (0, 0)      │
        □       │
                │
                │         ● cluster_3
        ┼───────┼─────────────────→ X+
                │    (0.875, -0.475)
                |
                │
                Y-
```

---

## 8. Key Achievements & Recent Fixes

### 8.1 Real YOLO Integration

**What:** Replaced mock_yolo_detector with real YOLO11 inference.

**Implementation:**
- Model: `best.pt` trained on Roboflow Cotton-boll-and-cluster-2 dataset
- Classes: `cotton_boll` (individual), `cotton_boll-cluster` (group)
- Confidence threshold: 0.7
- Library: ultralytics

**Two Service Endpoints:**
- `/yolo/detect` - Raw detections (all bounding boxes as-is)
- `/yolo/detect_clusters` - Pixel-space merged clusters (nearby bolls grouped)

**Image Saving:** Annotated images saved to `yolo_output/` for debugging:
- `detect_*.png` - Raw YOLO boxes
- `clusters_*.png` - Merged cluster bboxes
- `spatial_*.png` - 3D positions overlaid

### 8.2 Camera Intrinsics Bug Fix (P Matrix)

**Problem:** Detected cluster_2 appeared at Y=-0.18 instead of Y=0.0 (ground truth). Systematic ~20cm offset in all 3D projections.

**Root Cause Discovery:**
- Gazebo generated camera_info with **wrong P matrix** principal point
- P matrix had cx=160, cy=120 (should be 320, 240 for 640×480 image)
- K matrix was correct: cx=320, cy=240
- `PinholeCameraModel.projectPixelTo3dRay()` uses P matrix, not K matrix

**Attempted Fix #1:** Added `<lens><intrinsics>` to URDF with correct values:
```xml
<lens>
    <intrinsics>
        <fx>277</fx><fy>277</fy>
        <cx>320</cx><cy>240</cy>
    </intrinsics>
</lens>
```
→ Fixed K matrix, but P matrix remained wrong in Gazebo

**Final Fix:** Bypass PinholeCameraModel entirely in `depth_processor.py`:
```python
# Extract from K matrix directly
self.fx = msg.k[0]   # K[0,0]
self.fy = msg.k[4]   # K[1,1]
self.cx = msg.k[2]   # K[0,2]
self.cy = msg.k[5]   # K[1,2]

# Back-project manually
X = (u - cx) * depth / fx
Y = (v - cy) * depth / fy
Z = depth
```

**Result:** Error reduced from ~20cm to ~1-2cm

**Lesson Learned:** Always verify BOTH K and P matrices in camera_info. Gazebo may set them inconsistently. When debugging 3D projection errors, trace the full pipeline: pixel → camera frame → world frame.

### 8.3 Complete-Linkage Clustering

**Problem:** Initial single-linkage clustering created incorrect groupings. Example:
- Boll A close to B, B close to C, but A far from C
- Single-linkage: A-B-C grouped together (chain-linking artifact)
- Complete-linkage: A-B in one cluster, C in another (correct)

**Algorithm:**

```
Single-linkage (BAD):
    boll joins cluster if close to ANY member
    → chains form across distant bolls

Complete-linkage (GOOD):
    boll joins cluster if close to ALL members
    → tight, compact clusters only
```

**Implementation** (spatial_detection_pipeline.py):

```python
def is_close_to_all(new_pos, cluster):
    for existing in cluster.detections:
        if XY_distance(new_pos, existing) > merge_radius:
            return False
    return True
```

**Merge Radius Calculation:**
- Auto-calculated from ground truth: 25% of minimum inter-cluster distance
- Min distance: 0.485m (cluster_1 ↔ cluster_2)
- Merge radius: 0.485 × 0.25 = 0.121m

This ensures:
- Bolls on same plant → group together (within 12cm)
- Different plants → stay separate (48cm apart)

### 8.4 3D Accuracy Validation

**Ground Truth Comparison Framework:**

1. Run panoramic scan with detection enabled
2. Call `/detection/validate` service
3. Nearest-neighbor matching: each ground truth → closest detected cluster
4. Pass/fail based on tolerance (default: 5cm)

**Validation Results (after P matrix fix):**

| Ground Truth | Detected Position | Error |
|--------------|-------------------|-------|
| cluster_2 (0.975, 0.0, 0.52) | (0.970, 0.006, 0.474) | ~1-2cm |

**Why ~1-2cm Accuracy:**
- Camera intrinsics now correct (K matrix)
- TF chain accurate (robot_state_publisher)
- Depth sensor noise: σ=0.007 (configured in URDF)
- Focus iterations center target in view (reduces edge effects)

### 8.5 World-Space vs Pixel-Space Clustering

**Two Clustering Approaches Available:**

| Approach | Location | Distance Unit | Reliability |
|----------|----------|---------------|-------------|
| Pixel-space | real_yolo_detector | pixels | Unreliable at varying distances |
| World-space | spatial_detection_pipeline | meters | Reliable (uses depth + TF) |

**Why World-Space is Better:**
- Pixel distance varies with camera distance (perspective)
- At 0.5m, two bolls 10cm apart ≈ 55 pixels
- At 1.0m, same bolls ≈ 27 pixels
- World-space: always 10cm regardless of camera position

**Recommendation:** Use `/detection/run_at_position` (world-space) for accurate 3D localization. Use `/yolo/detect_clusters` (pixel-space) only for quick sanity checks.

---

## 9. How to Run

### 9.1 Prerequisites

**Operating System:** Ubuntu 22.04 (via WSL2 on Windows)

**ROS2 Packages:**
```bash
# ROS2 Humble (already installed)
sudo apt install ros-humble-desktop

# Gazebo Ignition Fortress
sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control

# ros2_control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# MoveIt 2
sudo apt install ros-humble-moveit

# Vision dependencies
sudo apt install ros-humble-cv-bridge ros-humble-image-geometry ros-humble-tf2-geometry-msgs
```

**Python Dependencies:**
```bash
# YOLO (ultralytics)
pip3 install ultralytics

# NumPy (must match cv_bridge version)
pip3 install numpy==1.24.3
```

**WSL2 Graphics Fix:**
```bash
# Add to ~/.bashrc for software rendering (avoids GPU issues)
export LIBGL_ALWAYS_SOFTWARE=1
```

### 9.2 Build

```bash
# Enter WSL
wsl -d Ubuntu-22.04

# Navigate to workspace
cd /mnt/c/Users/ayhan/harvesting_ws

# Build core packages
colcon build --packages-select harvester_interfaces robot_arm robot_arm_moveit_config orchestrator

# Source workspace
source install/setup.bash
```

**Incremental builds:**
```bash
# Just orchestrator (after code changes)
colcon build --packages-select orchestrator

# Full detection stack
colcon build --packages-select harvester_interfaces orchestrator robot_arm robot_arm_moveit_config
```

### 9.3 Launch Simulation

**Terminal 1 - Gazebo + Robot:**
```bash
cd /mnt/c/Users/ayhan/harvesting_ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch robot_arm bot.launch.py
```

This launches:
- Gazebo Sim with cotton_field world
- robot_state_publisher (TF + URDF)
- ros_gz_bridge (topic bridging)
- joint_state_broadcaster + arm_controller
- landmark_publisher (static TF + collision objects)

**Terminal 2 - MoveIt (optional, for motion planning):**
```bash
source install/setup.bash
ros2 launch robot_arm_moveit_config moveit.launch.py
```

This launches:
- move_group (OMPL motion planning)
- RViz with MotionPlanning plugin
- arm_commander (MoveIt action client)

### 9.4 Start Vision Pipeline

Run each node in a separate terminal (all after `source install/setup.bash`):

**Terminal 3 - YOLO Detector:**
```bash
ros2 run orchestrator real_yolo_detector
```

**Terminal 4 - Depth Processor:**
```bash
ros2 run orchestrator depth_processor
```

**Terminal 5 - Spatial Detection Pipeline:**
```bash
ros2 run orchestrator spatial_detection_pipeline
```

**Terminal 6 - Explorer (for scanning):**
```bash
ros2 run orchestrator explorer
```

**One-liner (background mode):**
```bash
ros2 run orchestrator real_yolo_detector &
ros2 run orchestrator depth_processor &
ros2 run orchestrator spatial_detection_pipeline &
ros2 run orchestrator explorer
```

### 9.5 Test Detection

**Test YOLO detection (raw):**
```bash
ros2 service call /yolo/detect harvester_interfaces/srv/YoloDetect "{}"
```

**Test pixel-to-3D conversion:**
```bash
ros2 service call /depth_processor/pixel_to_3d harvester_interfaces/srv/PixelTo3D "{u: 320, v: 240}"
```

**Run detection at current position:**
```bash
ros2 service call /detection/run_at_position std_srvs/srv/Trigger "{}"
```

**Validate against ground truth:**
```bash
ros2 service call /detection/validate std_srvs/srv/Trigger "{}"
```

**Print tracked clusters:**
```bash
ros2 service call /detection/print_results std_srvs/srv/Trigger "{}"
```

**Clear all detections:**
```bash
ros2 service call /detection/clear std_srvs/srv/Trigger "{}"
```

### 9.6 Run Panoramic Scan

**Full detection scan:**
```bash
# Clear previous detections
ros2 service call /detection/clear std_srvs/srv/Trigger "{}"

# Run 7×3 panoramic scan with detection at each position
ros2 service call /explorer/panoramic_scan std_srvs/srv/Trigger "{}"

# Print detection results
ros2 service call /detection/print_results std_srvs/srv/Trigger "{}"

# Validate against ground truth
ros2 service call /detection/validate std_srvs/srv/Trigger "{}"
```

**Fast scan (no detection):**
```bash
ros2 param set /explorer enable_detection false
ros2 service call /explorer/panoramic_scan std_srvs/srv/Trigger "{}"
```

**Arc sweep around known clusters:**
```bash
ros2 service call /explorer/start_scan std_srvs/srv/Trigger "{}"
```

### 9.7 Arm Control Commands

**Move to named position (via arm_commander):**
```bash
ros2 param set /arm_commander target_name cluster_1
ros2 service call /go_to_named std_srvs/srv/SetBool "{data: true}"
```

**Direct joint trajectory:**
```bash
# Move to position [hip, shoulder, elbow, wrist, l_gripper, r_gripper]
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['hip', 'shoulder', 'elbow', 'wrist', 'l_g_base', 'r_g_base'], \
    points: [{positions: [0.5, -0.5, 0.8, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"

# Return to home
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['hip', 'shoulder', 'elbow', 'wrist', 'l_g_base', 'r_g_base'], \
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"
```

**Close/Open gripper:**
```bash
# Close gripper
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['l_g_base', 'r_g_base'], \
    points: [{positions: [0.5, -0.5], time_from_start: {sec: 1}}]}"

# Open gripper
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['l_g_base', 'r_g_base'], \
    points: [{positions: [0.0, 0.0], time_from_start: {sec: 1}}]}"
```

### 9.8 Debug & Monitoring

**Check joint states:**
```bash
ros2 topic echo /joint_states --once
```

**List camera topics:**
```bash
ros2 topic list | grep camera
```

**View camera feed:**
```bash
ros2 run rqt_image_view rqt_image_view
# Select /camera/color/image_raw
```

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
# Creates frames.pdf
```

**List controllers:**
```bash
ros2 control list_controllers
```

**Check services:**
```bash
ros2 service list | grep -E "(yolo|detection|explorer|depth)"
```

---

## 10. File Structure

```
harvesting_ws/
├── src/
│   ├── robot_arm/                         # 4-DOF arm (Gazebo simulation)
│   │   ├── config/
│   │   │   ├── environment_config.yaml    # World bounds, clusters, landmarks
│   │   │   └── gz_bridge.yaml             # ROS↔Gazebo topic bridges
│   │   ├── launch/
│   │   │   └── bot.launch.py              # Main simulation launch
│   │   ├── models/
│   │   │   └── cotton_cluster/            # Cotton 3D models (DAE meshes)
│   │   │       ├── meshes/object_0/       # Cotton mesh + texture
│   │   │       ├── meshes/object_1/       # Alternative cotton mesh
│   │   │       ├── model.sdf              # Gazebo model definition
│   │   │       └── model.config           # Model metadata
│   │   ├── robot_arm/
│   │   │   └── landmark_publisher.py      # Static TF + collision objects
│   │   ├── scripts/
│   │   │   ├── setup_cotton_model.py      # Cotton model setup utility
│   │   │   └── setup_cotton_in_car.py     # Alternative setup script
│   │   ├── src/
│   │   │   ├── write_pos.py               # Interactive CLI arm control
│   │   │   └── tcp_monitor.py             # TCP position monitor (FK)
│   │   ├── urdf/
│   │   │   └── mybot.urdf.xacro           # Robot URDF (arm + camera + gripper)
│   │   ├── worlds/
│   │   │   └── cotton_field.world         # Gazebo world (3 plants)
│   │   ├── yaml/
│   │   │   └── controllers.yaml           # ros2_control configuration
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── robot_arm_moveit_config/           # MoveIt 2 configuration
│   │   ├── config/
│   │   │   ├── robot_arm.srdf             # Planning groups, end-effectors
│   │   │   ├── kinematics.yaml            # KDL IK solver config
│   │   │   ├── joint_limits.yaml          # Velocity/acceleration limits
│   │   │   ├── ompl_planning.yaml         # OMPL planner configs
│   │   │   ├── moveit_controllers.yaml    # MoveIt→ros2_control bridge
│   │   │   ├── ros2_controllers.yaml      # Controller definitions
│   │   │   └── moveit.rviz                # RViz config with MotionPlanning
│   │   ├── launch/
│   │   │   └── moveit.launch.py           # MoveIt launch (move_group + RViz)
│   │   ├── robot_arm_moveit_config/
│   │   │   └── arm_commander.py           # MoveIt action client node
│   │   └── package.xml
│   │
│   ├── orchestrator/                      # Vision pipeline + orchestration
│   │   ├── models/
│   │   │   └── best.pt                    # YOLO11 trained model
│   │   ├── orchestrator/
│   │   │   ├── real_yolo_detector.py      # Real YOLO inference node
│   │   │   ├── mock_yolo_detector.py      # Mock detector (for testing)
│   │   │   ├── spatial_detection_pipeline.py  # 2D→3D + clustering
│   │   │   ├── depth_processor.py         # Pixel→3D conversion
│   │   │   ├── camera_focus.py            # Camera centering control
│   │   │   ├── explorer.py                # Panoramic/arc scan controller
│   │   │   ├── main.py                    # (Legacy) Mock orchestrator
│   │   │   └── __init__.py
│   │   ├── test/                          # Unit tests
│   │   ├── package.xml
│   │   └── setup.py                       # Entry points for all nodes
│   │
│   ├── harvester_interfaces/              # Custom ROS2 interfaces
│   │   ├── msg/
│   │   │   ├── BoundingBox.msg            # YOLO detection bbox
│   │   │   └── DetectedCluster.msg        # 3D cluster with metadata
│   │   ├── srv/
│   │   │   ├── YoloDetect.srv             # Run YOLO detection
│   │   │   ├── PixelTo3D.srv              # Pixel→world conversion
│   │   │   ├── FocusFromPixel.srv         # Center camera on pixel
│   │   │   ├── FocusFromPosition.srv      # Center camera on 3D point
│   │   │   └── RunDetectionPipeline.srv   # Full detection pipeline
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── vision_ml/                         # (Legacy) Mock vision node
│   │   ├── vision_ml/main.py
│   │   └── ...
│   │
│   ├── robotic_actor/                     # (Legacy) Gripper simulation
│   │   ├── urdf/four_bar.urdf
│   │   ├── robotic_actor/main.py
│   │   └── ...
│   │
│   ├── logger_node/                       # System logging
│   │   ├── logger_node/main.py            # CSV logging node
│   │   └── ...
│   │
│   ├── example_arm_description/           # (Learning) 2-DOF example arm
│   │   ├── description/*.xacro
│   │   └── ...
│   │
│   ├── example_arm_moveit_config/         # (Learning) MoveIt example
│   │   ├── config/target_positions.yaml
│   │   └── ...
│   │
│   └── docs/
│       ├── CHECKPOINT.md                  # This file
│       ├── TASKS.md                       # Active task list
│       ├── session_notes.md               # Session summaries
│       ├── PLAN_YOLO_INTEGRATION.md       # YOLO integration plan
│       ├── commands.md                    # Quick reference commands
│       └── RESEARCH/
│           ├── Cotton-Tracking-YOLO/      # YOLO model + 3D GLB models
│           │   ├── best.pt                # Trained model
│           │   ├── object_0.glb           # Cotton 3D model #1
│           │   ├── object_1.glb           # Cotton 3D model #2
│           │   ├── sticky_tracker.yaml    # BoT-SORT config
│           │   ├── track_webcam_v3.py     # Webcam tracking script
│           │   └── Cotton-boll-and-cluster-2/  # Training dataset
│           │       └── data.yaml          # 2 classes: cotton_boll, cotton_boll-cluster
│           └── EI_Pick_n_Place/           # Pick-and-place reference
│               └── pnp_ws/src/
│                   ├── braccio_description/    # Braccio 6-DOF URDF
│                   ├── braccio_moveit_config/  # MoveIt config
│                   ├── ei_yolov5_detections/   # YOLOv5 ROS node
│                   └── pick_n_place/           # Pick-and-place logic
│
├── install/                               # Built packages (colcon)
├── build/                                 # Build artifacts
├── log/                                   # Build/run logs
└── yolo_output/                           # Detection visualization images
    ├── detect_*.png                       # Raw YOLO detections
    ├── clusters_*.png                     # Pixel-merged clusters
    └── spatial_*.png                      # 3D positions overlaid
```

---

## 11. Next Steps / Pending Tasks

### From TASKS.md:

1. **3D Cotton Models in Gazebo**
   - Clone Cotton-Tracking-YOLO repo (object_0.glb, object_1.glb)
   - GLB supported in Fortress
   - Replace current cotton cluster meshes

2. **CLUSTER_VIEW Implementation**
   - After panoramic scan finds clusters
   - Approach position for full cluster view
   - Camera focus iterations to center

3. **Boll Detection within Cluster**
   - From CLUSTER_VIEW, detect individual bolls
   - Store positions for sequential picking

4. **Pick-and-Place Integration**
   - Reference: EI_Pick_n_Place repo
   - Visual servoing approach
   - Gripper control

5. **Braccio 6-DOF Arm**
   - Replace 4-DOF with Braccio URDF
   - Better reachability for picking
   - here we should match the environment configurations we've defined in reports too.

6. **Full Cycle Demo**
   - Complete harvesting loop
   - Video recording for documentation

---

## 12. Quick Reference Commands

### Build
```bash
cd ~/harvesting_ws
colcon build --packages-select robot_arm orchestrator harvester_interfaces
source install/setup.bash
```

### Launch Gazebo Simulation
```bash
ros2 launch robot_arm bot.launch.py
```

### Start Vision Pipeline Nodes
```bash
# Terminal 2
ros2 run orchestrator real_yolo_detector

# Terminal 3
ros2 run orchestrator depth_processor

# Terminal 4
ros2 run orchestrator spatial_detection_pipeline

# Terminal 5
ros2 run orchestrator explorer
```

### Test Detection
```bash
# Raw YOLO detection
ros2 service call /yolo/detect harvester_interfaces/srv/YoloDetect "{}"

# Pixel to 3D conversion
ros2 service call /depth_processor/pixel_to_3d harvester_interfaces/srv/PixelTo3D "{u: 320, v: 240}"

# Run detection at current position
ros2 service call /detection/run_at_position std_srvs/srv/Trigger "{}"

# Validate against ground truth
ros2 service call /detection/validate std_srvs/srv/Trigger "{}"
```

### Run Panoramic Scan
```bash
# Clear previous detections
ros2 service call /detection/clear std_srvs/srv/Trigger "{}"

# Start panoramic scan with detection
ros2 service call /explorer/panoramic_scan std_srvs/srv/Trigger "{}"

# Print results
ros2 service call /detection/print_results std_srvs/srv/Trigger "{}"
```

### Debug
```bash
# Check joint states
ros2 topic echo /joint_states --once

# Check camera topics
ros2 topic list | grep camera

# Check TF
ros2 run tf2_tools view_frames

# List controllers
ros2 control list_controllers
```

---

## 13. Reference Materials

### Cotton-Tracking-YOLO Repository
- **Location:** `docs/RESEARCH/Cotton-Tracking-YOLO/`
- **Model:** best.pt (2 classes: cotton_boll, cotton_boll-cluster)
- **3D Models:** object_0.glb, object_1.glb
- **Trackers:** sticky_tracker.yaml (BoT-SORT with Re-ID)

### EI_Pick_n_Place Repository
- **Location:** `docs/RESEARCH/EI_Pick_n_Place/`
- **Reference for:** Visual servoing, pick-and-place algorithms
- **Arm:** Braccio 6-DOF

---

*Last updated: 2026-01-06*
