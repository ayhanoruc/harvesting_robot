# RoboCot Project Checkpoint — 2026-04-02

Cotton harvesting robot: Doosan M1013 6-DOF arm + Robotiq Hand-E gripper, simulated in Gazebo Ignition Fortress, controlled via ROS2 Humble + MoveIt2. YOLO-based cotton boll detection, depth-based 3D localization, automated pick-and-place cycle.

---

## Repository Structure

```
harvesting_ws/
├── src/                              # Git repo root (git is HERE, not in harvesting_ws/)
│   ├── robot_arm/                    # Package: Gazebo sim, URDF, world, controllers
│   ├── robot_arm_moveit_config/      # Package: MoveIt2 config + arm_commander
│   ├── orchestrator/                 # Package: All vision + harvest pipeline nodes
│   ├── harvester_interfaces/         # Package: Custom ROS2 msgs/srvs
│   └── docs/                         # Documentation, scripts, research
│       ├── tasks_today.md            # Current sprint TODO list
│       ├── CHECKPOINT.md             # This file
│       ├── parse_harvest_log.py      # Log parser for post-run analysis
│       ├── fov_calc.py               # FOV geometry calculator
│       └── RESEARCH/                 # YOLO training, papers, etc.
├── yolo_output/                      # YOLO annotated images + harvest logs (runtime)
├── install/                          # Colcon install space
└── build/                            # Colcon build space
```

---

## Packages & Key Files

### 1. `robot_arm` — Simulation Infrastructure

| File | Purpose |
|------|---------|
| `urdf/m1013_robocot.urdf.xacro` | Full URDF: M1013 6-DOF + Hand-E gripper + wrist RGB-D camera. Includes ros2_control hardware interfaces and gz_ros2_control plugin. |
| `worlds/cotton_field.world` | Gazebo world: 3 static plants (green stems + DAE cluster mesh), 3 dynamic cotton boll spheres (r=0.035m, static=false), 1 reservoir box. |
| `yaml/controllers.yaml` | ros2_control config: joint_state_broadcaster (7 joints), arm_controller (joint1-6), gripper_controller (left finger only — right mimics). |
| `config/environment_config.yaml` | Single source of truth: cluster positions, reservoir landmark, workspace bounds, collision objects. |
| `config/gz_bridge.yaml` | Gazebo-ROS bridges: clock, camera_info (color+depth). Image bridges handled separately via ros_gz_image. |
| `launch/bot.launch.py` | Launch: Gazebo + robot_state_publisher + spawn + bridges + controller spawners (sequenced) + landmark_publisher. |
| `robot_arm/landmark_publisher.py` | Publishes static TF frames for all landmarks + collision objects to /planning_scene. |

**URDF Critical Details:**
- Camera: on `link6`, joint to `camera_link` with `rpy="0 -1.5708 0"` (Gazebo camera looks along +X, rotated to align with tool0 +Z)
- Camera optical frame: `rpy="-1.5708 0 -1.5708"` (ROS convention: Z forward, X right, Y down)
- Camera: 640x480, 90 deg HFOV, 1 Hz update rate (sim performance)
- Left finger: axis="-1 0 0", prismatic 0-0.025m
- Right finger: axis="+1 0 0", prismatic 0-0.025m, `<mimic joint="hande_left_finger_joint" multiplier="1.0" offset="0.0"/>`
- Right finger has NO ros2_control command interface (driven by Gazebo mimic physics)
- tcp frame: 0.14m beyond tool0 along Z

**World Positions (Gazebo coordinates):**
- Robot base: (0, 0, 0)
- Plant 1 / cluster_3: (0.875, -0.475, stem_h=0.42)
- Plant 2 / cluster_2: (0.975, 0.0, stem_h=0.52)
- Plant 3 / cluster_1: (0.875, 0.475, stem_h=0.46)
- Dynamic bolls: at cluster tops (z = 0.50, 0.56, 0.46 respectively)
- Reservoir box: (0.4, 0.0, box 0.3x0.3x0.2)

### 2. `robot_arm_moveit_config` — Motion Planning

| File | Purpose |
|------|---------|
| `config/robot_arm.srdf` | MoveIt SRDF: arm group (base_0->tool0 chain), gripper group (left finger only), named poses (home/ready/zero/open/closed), collision matrix. |
| `config/kinematics.yaml` | KDL solver, 0.5s timeout, 20 attempts. |
| `config/ompl_planning.yaml` | OMPL planners: default=RRTstar for arm, RRTConnect for gripper. |
| `config/moveit_controllers.yaml` | MoveIt controller manager: arm_controller + gripper_controller (left finger only). |
| `config/joint_limits.yaml` | Joint velocity/acceleration limits. |
| `launch/moveit.launch.py` | Launch: move_group + RViz + arm_commander + gripper_controller_node. |
| `robot_arm_moveit_config/arm_commander.py` | **Core motion node.** See detailed section below. |

### 3. `orchestrator` — Vision + Harvest Pipeline

| File | Purpose |
|------|---------|
| `orchestrator/main.py` | **Top-level state machine.** IDLE->SCANNING->APPROACHING->HARVESTING->RETURNING->IDLE |
| `orchestrator/harvest_executor.py` | **8-step pick-and-place.** Pre-grasp->open->approach->close->retract->reservoir->release->return |
| `orchestrator/explorer.py` | Panoramic scan: 3 j1 positions from HOME, joint-space trajectory. Also has arc sweep mode (unused in demo). |
| `orchestrator/real_yolo_detector.py` | YOLO inference on camera frames. Two services: /yolo/detect (raw) and /yolo/detect_clusters (merged). Saves annotated PNGs. |
| `orchestrator/depth_processor.py` | Pixel->3D: back-projects (u,v) using depth + K matrix, transforms camera_optical_frame->world via TF. |
| `orchestrator/camera_focus.py` | Pixel-error proportional control: adjusts j1/j2/j3 to center target pixel. Used by spatial_detection_pipeline (0 iterations in sim). |
| `orchestrator/spatial_detection_pipeline.py` | Coordinates YOLO->focus->depth->cluster tracking. Services: run_at_position, get_results, validate, clear. |
| `orchestrator/gripper_controller.py` | Open/close services via JointTrajectory topic publish + joint state polling. Left finger only (right mimics). |
| `orchestrator/mock_yolo_detector.py` | Synthetic detection for testing without YOLO model. |
| `launch/harvest_pipeline.launch.py` | Launch: all 7 pipeline nodes with parameters. |

### 4. `harvester_interfaces` — Custom Messages & Services

| Interface | Fields |
|-----------|--------|
| `msg/BoundingBox` | u_min, v_min, u_max, v_max, confidence, label, area |
| `msg/DetectedCluster` | cluster_id, position (Point), confidence, best_bbox_area, num_detections, best_scan_position |
| `srv/YoloDetect` | Request: (empty) -> Response: detections[], success, message |
| `srv/PixelTo3D` | Request: u, v -> Response: position (Point), success, message |
| `srv/HarvestBoll` | Request: boll_position, pre_grasp_position -> Response: success, message |
| `srv/GetDetectedClusters` | Request: (empty) -> Response: clusters[], success, message |
| `srv/FocusFromPixel` | Request: u, v -> Response: success, message |

---

## Node Architecture & Service Map

```
Terminal 1: bot.launch.py
  |-- Gazebo Ignition (cotton_field.world)
  |-- robot_state_publisher (URDF -> TF)
  |-- ros_gz_bridge (clock, camera_info)
  |-- ros_gz_image (color + depth images)
  |-- joint_state_broadcaster
  |-- arm_controller (JointTrajectoryController, 6 joints)
  |-- gripper_controller (JointTrajectoryController, left finger)
  +-- landmark_publisher (static TFs + collision objects)

Terminal 2: moveit.launch.py
  |-- move_group (MoveIt2 planning + execution)
  |-- rviz2
  |-- arm_commander (IK + joint goals)
  +-- gripper_controller_node (open/close services)

Terminal 3: harvest_pipeline.launch.py
  |-- explorer (panoramic scan, 3 positions)
  |-- real_yolo_detector (YOLO inference)
  |-- depth_processor (pixel -> 3D)
  |-- camera_focus (pixel error -> joint adjust)
  |-- spatial_detection_pipeline (detection coordination)
  |-- harvest_executor (single-boll pick-place)
  +-- orchestrator_node (state machine)
```

### Service Call Map

```
orchestrator_node
  |-- /explorer/panoramic_scan (Trigger) -> explorer
  |-- /detection/clear (Trigger) -> spatial_detection_pipeline
  |-- /detection/get_results (GetDetectedClusters) -> spatial_detection_pipeline
  |-- /yolo/detect (YoloDetect) -> real_yolo_detector [boll-level]
  |-- /depth_processor/pixel_to_3d (PixelTo3D) -> depth_processor
  |-- /go_to_pose (SetBool) -> arm_commander
  |-- /go_to_named (SetBool) -> arm_commander
  |-- /arm_commander/set_parameters (SetParameters) -> arm_commander
  |-- /harvest/pick_boll (HarvestBoll) -> harvest_executor
  +-- /gripper/open (Trigger) -> gripper_controller

harvest_executor
  |-- /go_to_pose (SetBool) -> arm_commander
  |-- /arm_commander/set_parameters (SetParameters) -> arm_commander
  |-- /gripper/open (Trigger) -> gripper_controller
  +-- /gripper/close (Trigger) -> gripper_controller

explorer (during panoramic scan)
  |-- /arm_controller/joint_trajectory (topic publish, DIRECT — no path planning)
  |-- /detection/clear (Trigger) -> spatial_detection_pipeline
  |-- /detection/wait_ready (Trigger) -> spatial_detection_pipeline
  +-- /detection/run_at_position (Trigger) -> spatial_detection_pipeline

spatial_detection_pipeline
  |-- /yolo/detect_clusters (YoloDetect) -> real_yolo_detector [cluster-level]
  |-- /depth_processor/pixel_to_3d (PixelTo3D) -> depth_processor
  +-- /camera_focus/center_on_pixel (FocusFromPixel) -> camera_focus [0 iters in sim]

arm_commander
  |-- /compute_ik (GetPositionIK) -> MoveIt move_group
  +-- move_action (MoveGroup action) -> MoveIt move_group [PATH PLANNING via RRTstar]
```

---

## arm_commander.py — Detailed

**Startup sequence:**
1. Wait for MoveGroup action server + /compute_ik service
2. Wait for joint states
3. Go to HOME_JOINTS = [0.0, -0.922, 2.4494, 0.0, -1.3, 0.0]
4. Rotate joint5 by `cluster_rotate_deg` (90 deg) -> j5 becomes ~0.27 -> camera faces clusters

**IK Pipeline:**
1. `compute_approach_quaternion(x,y,z)` -> quaternion pointing tcp Z from base toward (x,y) horizontally
2. `compute_ik_multi_seed(x,y,z,orientation)` -> try HOME seed + current seed, pick lowest-cost solution
3. `_normalize_joints()` -> wrap to shortest path (+-pi from current)
4. `validate_joints()` -> reject if j1 > 135 deg (backward reach)
5. `send_joint_goal()` -> MoveGroup action (uses OMPL RRTstar for path planning)
6. `_check_tcp_error()` -> verify <5cm, retry via HOME if >5cm

**KEY ISSUE (active):** `send_joint_goal` uses MoveGroup action which triggers OMPL path planning (RRTstar, 5s timeout). This is unnecessary since IK already gives us the target joints. Should be replaced with direct JointTrajectory publish (like explorer does) for speed and reliability.

**Services:**
- `/go_to_pose` — reads target_x/y/z + use_approach_orientation params, runs IK pipeline
- `/go_to_named` — loads position from config, applies pre_grasp_offset for clusters, runs IK pipeline
- `/go_home_view` — HOME joints + j5 rotation (cluster-facing view)
- `/rotate_home_view_to_clusters` — j5 rotation only

**HOME positions:**
- `HOME_JOINTS` = [0.0, -0.922, 2.4494, 0.0, -1.3, 0.0] — base home, camera NOT facing clusters
- After startup rotation: j5 = -1.3 + pi/2 = 0.2708 — camera faces clusters
- `go_to_named('home')` sends HOME_JOINTS WITHOUT rotation (reverts to non-cluster-facing)

---

## Harvest Cycle — Full Data Flow

### Trigger
```bash
ros2 service call /orchestrator/start_harvest std_srvs/srv/Trigger "{}"
```

### Phase 1: SCANNING
1. Clear previous detections (`/detection/clear`)
2. Start panoramic scan (`/explorer/panoramic_scan`) — spawns thread, returns immediately
3. Explorer thread: for each of 3 positions (j1 = -0.50, 0.0, +0.50 rad, j5 = 0.2708):
   - Publish JointTrajectory to `/arm_controller/joint_trajectory` (DIRECT, no MoveIt)
   - Wait move + pause duration
   - Call `/detection/run_at_position` -> spatial_detection_pipeline:
     - `/yolo/detect_clusters` -> YOLO inference -> group nearby bolls -> cluster bboxes
     - For each cluster bbox: `/depth_processor/pixel_to_3d` -> 3D world position
     - Add to tracked_clusters (world-space complete-linkage merge)
4. Explorer returns to cluster-facing HOME (rotated j5), publishes COMPLETE
5. Orchestrator gets results via `/detection/get_results`
6. **For demo: always uses config positions regardless of detection results** (vision runs for image capture)

### Phase 2: APPROACHING (per cluster)
1. Compute pre-grasp: 15cm back from cluster center along approach vector
2. `_go_to_xyz(pre_grasp, approach_orientation=True)` -> arm_commander IK -> MoveGroup
3. Wait 3s camera settle time
4. `/yolo/detect` (raw bolls) -> for each: `/depth_processor/pixel_to_3d` -> boll 3D positions
5. Fallback: if no bolls detected, use cluster center

### Phase 3: HARVESTING (per boll)
8-step sequence via harvest_executor:
1. **PRE-GRASP**: go to pre-grasp (approach_orientation=True)
2. **OPEN**: /gripper/open -> fingers spread (position 0.0)
3. **APPROACH**: go to boll (3cm standoff, approach_orientation=True)
4. **CLOSE**: /gripper/close -> fingers close (position 0.025)
5. **RETRACT**: go to pre-grasp (safer than lifting at workspace edge)
6. **RESERVOIR**: go to (0.4, 0.0, 0.35) — hover 15cm above box top
7. **RELEASE**: /gripper/open
8. **RETURN**: go to pre-grasp (approach_orientation=True)

### Phase 4: RETURNING
- `_go_home()` -> arm_commander sends HOME_JOINTS (non-rotated)

---

## Gripper Details

- **Left finger**: hande_left_finger_joint, axis -X, prismatic 0->0.025m
- **Right finger**: mimic of left (URDF `<mimic>` tag), NO ros2_control interface
- **Open** = position 0.0 (fingers spread ~4.5cm total)
- **Close** = position 0.025 (fingers nearly touching)
- **Control**: gripper_controller_node publishes JointTrajectory to `/gripper_controller/joint_trajectory`
- **Feedback**: polls /joint_states for left finger position, 2mm tolerance, 60s timeout
- **Known**: boll diameter 70mm > Hand-E opening ~50mm -> mock grip for demo

---

## Camera & Vision Pipeline

**Camera specs**: 640x480, 90 deg HFOV (~74 deg VFOV), 1 Hz in sim, on link6
**YOLO model**: `src/orchestrator/models/best.pt`, classes: cotton_boll, cotton_boll-cluster, unripe-cotton
**Confidence threshold**: 0.5 (launch param)

**Two detection modes:**
1. `/yolo/detect_clusters` — merges nearby boll bboxes into cluster bboxes (pixel-space, 150px threshold). Used during panoramic scan.
2. `/yolo/detect` — returns raw individual detections. Used for boll-level detection at cluster view.

**Depth**: K matrix back-projection (fx=fy=277, cx=320, cy=240), TF camera_optical_frame->world
**Images saved to**: `yolo_output/` (detect_*.png, clusters_*.png, spatial_*.png)

---

## Configuration — environment_config.yaml

```yaml
clusters:
  cluster_3: {position: [0.875, -0.475, 0.46]}   # Right
  cluster_2: {position: [0.975, 0.0, 0.56]}       # Center
  cluster_1: {position: [0.875, 0.475, 0.50]}     # Left

landmarks:
  reservoir: {position: [0.4, 0.0, 0.3]}          # Front of robot
  explore_start: {position: [0.5, 0.50, 0.65]}
  explore_end: {position: [0.5, -0.50, 0.65]}

collision_objects: {}  # Reservoir collision DISABLED for reachability
```

---

## Launch & Test Commands

```bash
# Build all
colcon build --packages-select harvester_interfaces robot_arm robot_arm_moveit_config orchestrator
source install/setup.bash

# Terminal 1 — Gazebo
ros2 launch robot_arm bot.launch.py

# Terminal 2 — MoveIt + arm
ros2 launch robot_arm_moveit_config moveit.launch.py

# Terminal 3 — Pipeline (with log recording)
ros2 launch orchestrator harvest_pipeline.launch.py 2>&1 | tee yolo_output/harvest.log

# Trigger harvest
ros2 service call /orchestrator/start_harvest std_srvs/srv/Trigger "{}"

# Monitor
ros2 topic echo /orchestrator/status
ros2 topic echo /orchestrator/progress

# Emergency stop
ros2 service call /orchestrator/stop std_srvs/srv/Trigger "{}"

# Parse logs after run
python3 src/docs/parse_harvest_log.py yolo_output/harvest.log

# Copy results to desktop
DEST="C:/Users/ayhan/Desktop/harvest_demo_$(date +%Y%m%d_%H%M%S)" && mkdir -p "$DEST" && cp -r /mnt/c/Users/ayhan/harvesting_ws/yolo_output/*.png "$DEST/" 2>/dev/null; cp /mnt/c/Users/ayhan/harvesting_ws/yolo_output/harvest.log "$DEST/" 2>/dev/null; python3 /mnt/c/Users/ayhan/harvesting_ws/src/docs/parse_harvest_log.py "$DEST/harvest.log" > "$DEST/summary.txt" 2>/dev/null; echo "Saved to $DEST"
```

---

## Known Issues & Active Bugs

| Issue | Status | Detail |
|-------|--------|--------|
| **Path planning slow/fails** | ACTIVE | `send_joint_goal` uses MoveGroup->RRTstar (5s timeout). Should use direct JointTrajectory publish. Causes reservoir reach failures. |
| **Reservoir unreachable** | PARTIALLY FIXED | Moved to (0.4, 0.0). Still uses MoveGroup path planning which can timeout. |
| **Right gripper finger** | FIXED | Added `<mimic>` tag to URDF, removed from ros2_control. Gazebo physics drives it. |
| **HOME view inconsistency** | FIXED | Removed _go_home before scan. Scan uses rotated j5=0.2708. But _go_home at cycle end still reverts to non-rotated HOME. |
| **Sim speed** | KNOWN | ~3-4% realtime in WSL2 (no GPU). Full cycle takes 5-10min wall clock. |
| **Cotton boll size** | KNOWN | 70mm diameter vs Hand-E ~50mm opening. Mock grip for demo. |
| **Depth failures** | OBSERVED | Some boll detections fail pixel_to_3d (depth=NaN at edges). Bolls with valid depth proceed. |

---

## Executor Pattern

Both `main.py` and `harvest_executor.py` use `MultiThreadedExecutor(num_threads=4)` with `ReentrantCallbackGroup`. Service calls use `_wait_future()` poll-wait (NOT `rclpy.spin_until_future_complete` which steals nodes from executors).

`arm_commander.py` uses `rclpy.spin()` (single-threaded) with `rclpy.spin_until_future_complete()` — safe for single-threaded executor.

---

## Scan Parameters (harvest_pipeline.launch.py)

```python
pan_joint1_angles: [-0.50, 0.0, 0.50]   # 3 pan positions (~29 deg each)
pan_joint2_range: [-0.922]                # HOME shoulder
pan_joint3_range: [2.4494]                # HOME elbow
pan_joint4_range: [0.0]
pan_joint5_range: [0.2708]                # ROTATED j5 (faces clusters)
pan_joint6_range: [0.0]
pan_pause_duration: 2.0                   # Seconds at each position
pan_move_duration: 1.5                    # Seconds to move between
```

---

## Next Priority: Replace MoveGroup with Direct Trajectory

The single biggest reliability improvement: replace `arm_commander.send_joint_goal()` (MoveGroup action -> OMPL RRTstar) with direct JointTrajectory publish to `/arm_controller/joint_trajectory`. The IK already gives us target joints — path planning is redundant and causes timeouts.

Reference implementation: `explorer._move_to_joints_sync()` already works this way.
