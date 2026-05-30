# RoboCot Field Deployment Specification

**Purpose:** Reference document for final presentation Q&A and downstream
mechanical / dynamic analysis (handed to teammate for ANSYS/ADAMS/etc.).

This is a **specification + open-questions** doc. Section 1 is the system
overview we already have. Section 2 enumerates the 10 critical questions
graders are most likely to ask; each one is **unanswered**, but lists every
data point already on hand (`GIVEN:` blocks) so the teammate doesn't have
to re-derive baseline numbers.

---

## 1. System Overview

### 1.1 Hardware Stack

| Layer | Component | Notes |
|---|---|---|
| Mobile base | **Clearpath Husky** (A200-class, simulated) | DiffDrive skid-steer; 4 wheels |
| Manipulator | **Doosan M1013** 6-DOF arm | Mounted on deck plate |
| End-effector | **Robotiq Hand-E** parallel gripper | Symmetric jaws; finger stroke 0–25mm |
| Sensor | RGB-D wrist camera | Eye-in-hand, mounted on `link6` |
| Storage | Open-top reservoir bin | Mounted at rear of deck |
| Compute (sim) | x86 dev PC, Ubuntu 22.04 + ROS 2 Humble | WSL2 host |
| Compute (target) | NVIDIA Jetson Orin (planned) | Onboard, off-board uplink TBD |

### 1.2 Mass & Dimensions

Values come from the URDF (`husky_robocot.urdf.xacro`,
`m1013_robocot.urdf.xacro`). Sim masses are nominal; manufacturer values
are noted where they differ from sim.

| Item | Sim mass (kg) | Real mass (kg) | Source |
|---|---|---|---|
| Husky body | 50 | 50 (A200) | URDF / Clearpath datasheet |
| Wheel (×4) | 2 | 2 | URDF |
| Mount plate | 2 | ~2 | URDF |
| M1013 arm | — (URDF inertias only) | **33** | Doosan datasheet |
| Hand-E gripper | — | **1.1** | Robotiq datasheet |
| Reservoir bin (empty) | 2 | ~2 (target) | URDF |
| Boll (ripe) | ~0.005 | ~5–8 g (typ. cotton boll) | Generated |
| Boll (unripe) | ~0.003 | — | Generated |
| **Total static (empty bin)** | **~98** | **~88** | sum above |
| **Total + 200 bolls in bin** | **~99** | **~89** | bolls negligible |

Husky body dimensions (URDF):

| Dimension | Value (m) |
|---|---|
| Length × Width × Height | 0.99 × 0.67 × 0.30 |
| Body bottom z above ground | 0.13 |
| Body CoG height above ground | 0.28 |
| Wheel radius | 0.165 |
| Wheel width | 0.10 |
| Wheel track (L–R separation) | 0.555 |
| Wheel base (F–R separation) | 0.51 |
| Deck top z (mount surface) | 0.43 |
| Mount plate height | 0.05 |
| Arm base (`base_0`) z | 0.48 |
| Reservoir local position | (−0.30, 0, +0.10) on deck |
| Reservoir interior LxWxH | 0.4 × 0.4 × 0.2 |
| Reservoir top z (world, level) | ~0.63 |
| Arm mount horizontal offset from husky_base | (+0.20, 0) |

M1013 + Hand-E geometry:

| Item | Value |
|---|---|
| Reach (nominal, datasheet) | 1.300 m |
| Reach (with Hand-E + TCP) | ~1.4 m |
| DOF | 6 (revolute) |
| Repeatability (manufacturer) | ±0.03 mm |
| Joint limit positions (URDF) | joint1,2,4,5,6: ±2π ; joint3: ±2.7925 rad (±160°) |
| Joint max velocities (datasheet) | joint1–3: 180°/s ; joint4–6: 360°/s |
| Joint max torques (datasheet, peak) | joint1–3: 346 Nm ; joint4–6: 96 Nm |
| HOME pose joints (rad) | [0, −0.922, 2.4494, 0, −1.3, 0] |
| SCOUT pose (post-init) | HOME with `joint1=−π/2`, `joint5=+π/2` |

### 1.3 Sensor Specs (wrist RGB-D camera)

URDF sensor block (Gazebo Ignition `rgbd_camera` plugin):

| Param | Value |
|---|---|
| Type | `rgbd_camera` (RGB + depth) |
| Update rate | 10 Hz (sim; capped from 30 Hz to avoid software-render tearing) |
| Image resolution | 640 × 480 |
| Horizontal FOV | 1.57 rad (90°) |
| Format | R8G8B8 (color), 32FC1 (depth) |
| Focal length (px) | fx = fy = 277 |
| Principal point (px) | cx = 320, cy = 240 |
| Mounting | Rigid on `camera_link`, child of `link6` |
| Optical frame | `camera_optical_frame` |
| Depth range used (filter) | 0.20 m – 3.00 m |

### 1.4 Simulation Environment

| Parameter | Value |
|---|---|
| Engine | Gazebo Ignition Fortress |
| ROS 2 distro | Humble |
| Physics step | 5 ms (sim default) |
| Controller update | 10 ms |
| World file | `robot_arm/worlds/orchard.world` (+ auto-generated `orchard_bolls.world`) |
| Trees | 236 across 11 rows |
| Tree bounding box (m) | X ∈ [4, 41], Y ∈ [3.6, 34.3] |
| Row pitch (Y axis) | ~3 m (inter-row aisle width) |
| In-row tree pitch (X) | mean 1.5 m, median 1.5 m, std 0.67 m |
| Canopy Z range | 0.4 – 2.0 m |
| Bolls generated | 177 across 30 trees |
| Boll radius (sim) | ripe = 0.035 m ; unripe = 0.025 m |
| Boll color | ripe: cream/white (rgba 0.95, 0.95, 0.9) ; unripe: olive-green |
| Boll physics | static visual (no rigid-body); mock-gripped via Gazebo `set_pose` |
| Husky default spawn | (15.8, 4.85, z=0, yaw=0) — aisle for row 0, front along +X (row line) |
| Reservoir parent | rigid to `husky_base_link` via deck composition |

### 1.5 Coordinate Frames (TF tree)

```
world
 └── odom                       (static; published at launch, set to spawn pose)
      └── husky_base_link       (dynamic; DiffDrive plugin → odom topic + TF)
           ├── front_left_wheel,front_right_wheel,rear_left_wheel,rear_right_wheel
           ├── deck_link        (fixed; CoG reference of platform top)
           │    ├── arm_mount_link    (fixed; mount plate)
           │    │    └── base_0       (M1013 arm base)
           │    │         └── link1..link6
           │    │              └── tool0
           │    │                   ├── hand_e_link
           │    │                   │    ├── hande_left_finger
           │    │                   │    └── hande_right_finger
           │    │                   ├── camera_link
           │    │                   │    └── camera_optical_frame
           │    │                   └── tcp                    (TCP marker)
           │    └── reservoir_link     (fixed; harvest target frame)
```

Conventions:
- `world` = global, gravity-aligned, +Z up.
- `odom` follows the same axes as `world`; set to spawn pose at launch so
  the TF chain stays consistent regardless of Husky drift.
- `camera_optical_frame` follows ROS optical convention: +Z forward,
  +X right, +Y down (image coords).
- `tcp` is offset along Hand-E centerline at finger midpoint.

### 1.6 Software Architecture

Pipeline is **strictly layered**; each layer composes services from
lower layers. Adding a new high-level behavior (e.g., row navigation)
only requires a new top-layer node — atomic services unchanged.

```
LEVEL 3 — System pipeline
  row_navigator                                    /row_nav/run
        │
        ├── _drive_to(x, y, yaw)        (closed-loop cmd_vel + TF P-control)
        └── /cluster_harvester/run
                │
LEVEL 2 — Single-cluster pipeline                   /cluster_harvester/run
  cluster_harvester
        │
        ├── /cluster_scan/run            ← cluster_scanner
        ├── parse JSON output
        ├── match detection XYZ → YAML model IDs (sim-only artifact)
        ├── sort by reach distance (TF world←base_0)
        └── /simple_harvest/start        ← simple_cluster_harvester
                │
LEVEL 1 — Subroutines
  cluster_scanner                                   /cluster_scan/run
        │
        ├── sweep arm through 3×4 pan/tilt grid (12 poses)
        ├── per pose: /yolo/detect → bboxes
        ├── per bbox centroid: /depth_processor/pixel_to_3d → world XYZ
        ├── deduplicate by 3D proximity (5 cm)
        ├── gap-rule cluster bounding (largest contiguous group)
        └── save JSON + top-down PNG

  simple_cluster_harvester                          /simple_harvest/start
        │
        ├── load runtime boll IDs (or fall back to ground-truth yaml)
        ├── per boll: 8-step pick
        │     1. arm_commander.go_to_xyz(boll)
        │     2. mock grip close
        │     3. teleport boll → TCP via Gazebo set_pose
        │     4. start carry thread (boll follows TCP @ 10 Hz)
        │     5. arm_commander.go_to_reservoir (heuristic 3-stage)
        │     6. stop carry, mock grip open
        │     7. teleport boll → reservoir bin (grid scatter)
        └── reservoir-carry thread keeps dropped bolls glued to bin as Husky moves
                │
LEVEL 0 — Atomic ROS services
  arm_commander
        ├── /go_to_pose                  IK + joint goal, position target
        ├── /go_to_named                 IK to named target ('home', etc.)
        ├── /go_to_reservoir             3-stage heuristic: HOME → joint1=π → IK
        ├── /go_home_view                HOME + STEP-2 wrist tilt
        └── moves arm via MoveGroup action (RRTstar OMPL planner)

  gripper_controller                     /gripper/open, /gripper/close

  cv_boll_detector                       /yolo/detect, /yolo/detect_clusters
        ├── input: /camera/color/image_raw + /camera/depth/image_raw
        └── physics-based classical CV (depth-first → color → shape → reach)

  depth_processor                        /depth_processor/pixel_to_3d
        ├── depth-back-project pixel → camera frame
        └── TF camera_optical_frame → world
```

#### State machine (per `/row_nav/run`)

```
IDLE
  │ Trigger
  ▼
ROUTE_LOOP — for each tree_id in route:
  │
  ▼
  DRIVE_TO_SCOUT (closed-loop, cmd_vel + TF feedback)
    │ pose-controller: turn-in-place if heading misaligned,
    │ then forward at min(0.8·dist, max_lin), then final yaw align
    │ exits on (dist < pos_tol AND |yaw_err| < yaw_tol) or timeout
    ▼
  CLUSTER_HARVEST_LOOP (max_iterations=3)
    │
    ▼
    SCAN (cluster_scanner.run)
      │ 12-pose pan/tilt sweep
      │ per pose: detect bolls → back-project XYZ → accumulate
      │ dedup by 5 cm radius, gap-rule cluster boundary
      ▼
    CONTINUATION DECISION
      │ if cluster_bolls == 0 → break (DONE for this cluster)
      ▼
    MATCH + SORT (detection xyz → YAML IDs, sort by reach)
      │
      ▼
    PICK_LOOP — for each boll (closest first):
      │
      ▼
      GO_TO_BOLL → MOCK_CLOSE → CARRY_START → GO_TO_RESERVOIR
        → MOCK_OPEN → DROP_INTO_BIN
      │
      ▼ next boll, or batch end
    ────────────────────────────
    re-enter SCAN (continuation re-check)
  │ all iterations done
  ▼ next tree_id in route
DONE — emit summary on /row_nav/status
```

#### ROS interface summary

| Service | Type | Layer |
|---|---|---|
| `/row_nav/run` | std_srvs/Trigger | 3 |
| `/cluster_harvester/run` | std_srvs/Trigger | 2 |
| `/cluster_scan/run` | std_srvs/Trigger | 1 |
| `/simple_harvest/start` | std_srvs/Trigger | 1 |
| `/yolo/detect` | harvester_interfaces/YoloDetect | 0 |
| `/yolo/detect_clusters` | harvester_interfaces/YoloDetect | 0 |
| `/depth_processor/pixel_to_3d` | harvester_interfaces/PixelTo3D | 0 |
| `/go_to_pose` | std_srvs/SetBool | 0 |
| `/go_to_named` | std_srvs/SetBool | 0 |
| `/go_to_reservoir` | std_srvs/SetBool | 0 |
| `/gripper/open`, `/gripper/close` | std_srvs/Trigger | 0 |

Topics: `/cmd_vel` (Twist), `/odom`, `/joint_states`, `/tf`,
`/camera/color/image_raw`, `/camera/depth/image_raw`,
`/camera/{color,depth}/camera_info`,
`/row_nav/status`, `/cluster_harvester/status`,
`/cluster_scan/status`, `/simple_harvest/status`.

#### Observed pipeline timing (sim)

| Stage | Wall-clock (s) |
|---|---|
| One pose move (MoveGroup, OMPL RRTstar) | 3–10 |
| Cluster scan (12 poses) | 25–35 |
| Single boll pick (go + reservoir + drop) | 60–120 |
| Cluster harvest iter (4 bolls + scans) | 5–8 min |
| Drive 1 m via cmd_vel (sim) | 4–8 |
| Full row (3 clusters) | ~25 min |

---

## 2. Critical Questions — Awaiting Mechanical / Dynamic Analysis

Each subsection begins with the question, then a `GIVEN:` block summarizing
every relevant data point already on hand. Answers will be filled in
after dynamic analysis is performed by teammate.

### Q1 — Husky payload envelope and CoG shift under deployment loading

> Does the Husky platform accept the full deployment payload within its
> rated envelope, and how does the system center-of-gravity move from
> baseline to fully laden?

**GIVEN:**
- Husky A200 published payload rating: **75 kg** (Clearpath).
- Total static deployed mass (M1013 + Hand-E + mount + reservoir empty)
  ≈ **38 kg** above the Husky body → within envelope, with margin
  ~37 kg for sensors, compute, batteries, harvested bolls, optional
  outriggers.
- Husky body CoG (URDF): centered, **z = 0.28 m** above ground.
- Arm base z = 0.48 m; arm CoG estimate (M1013 datasheet typical):
  when folded at HOME, ~0.8 m above mount = **z ≈ 1.0 m**; at full
  reach laterally, arm CoG moves outward by ~0.5–0.7 m.
- Reservoir CoG: deck-mounted, x = −0.30 m local (rear-biased), z ≈
  0.55 m. Mass empty 2 kg; 200 ripe bolls ≈ 1.2 kg added → negligible.
- Combined deployed CoG (back-of-envelope, static, arm folded):
  ~(0.05, 0, **0.5**) in Husky frame — moderately elevated vs.
  baseline 0.28 m, raising tipping susceptibility.

### Q2 — Static and quasi-static tipping margin at worst-case arm pose

> At the worst-case arm extension during a harvest cycle, what is the
> static stability margin? At what tilt angle does tipping begin?

**GIVEN:**
- Support polygon (wheel contact patches) approximated as a rectangle
  of **0.51 × 0.555 m** under the body (wheel_base × wheel_track),
  centered on `husky_base_link` origin.
- Arm max reach 1.3 m + Hand-E ≈ **1.4 m TCP from base_0**, which is
  1.4 m + 0.2 m mount offset = up to **1.6 m TCP from husky_base_link
  origin** in the worst direction.
- Sim test cases observed: TCP reaches (15.7, 4.7, 1.28) when Husky is
  at (15.8, 4.85, yaw=0) — i.e., ~0.4 m forward + side from base while
  carrying ~3 cm-radius "boll" mass (visual only).
- Reservoir reach is **behind** the Husky in current geometry —
  reservoir_link at deck `(-0.30, 0)`. Arm extends rearward during
  reservoir drop, lifting front wheels' contact pressure.
- Joint 1 rotated ±π during reservoir reach (heuristic) — full base
  rotation of the arm CoG around vertical axis: any direction
  is reachable, so analysis must check **all four worst sectors**
  (front, back, left, right) of arm extension.
- Suggested check: ZMP / static stability margin computation with
  combined Husky + arm + reservoir CoG at HOME, scout, pick, and
  reservoir-drop poses.

### Q3 — Joint velocity and torque envelope vs. sim defaults

> Are the sim joint velocity / acceleration scalings realistic for a
> deployed M1013, and what payload-at-reach torque envelope must be
> respected on real hardware?

**GIVEN:**
- Sim `arm_commander` parameters:
  - `max_velocity_scaling_factor = 1.0`
  - `max_acceleration_scaling_factor = 1.0`
- M1013 datasheet joint velocity caps (manufacturer):
  - joints 1–3: **180°/s = π rad/s**
  - joints 4–6: **360°/s = 2π rad/s**
- M1013 peak joint torques (manufacturer):
  - joints 1–3: **346 Nm**
  - joints 4–6: **96 Nm**
- URDF joint limit blocks (effort, velocity):
  - joint1: `effort=346, velocity=2.0944` (≈120°/s)
  - joint2,3,4,5,6: identical effort/velocity entries
- Sim payload at TCP: visual-only (no rigid body); real boll mass
  estimate 5–10 g per ripe boll, gripper closure force not modeled.
- Hand-E grip force: 20–185 N selectable; sim runs with
  `gripper_demo_bypass = True` (no real force).
- Observed sim trajectory: smooth, no slip; controllers in Gazebo are
  PID + JointTrajectoryController, gains tuned for ~3–10 s per move.

### Q4 — Base reaction during arm motion (micro-shift of TF chain)

> When the arm executes a fast joint motion (especially joint 1 rotation
> for the reservoir heuristic), does the Husky base shift enough to
> invalidate prior TF or require re-localization?

**GIVEN:**
- Arm mounted rigidly to deck; deck CoG above wheel base means joint 1
  rotation at high velocity produces a torque about Husky vertical axis.
- Joint 1 swing of π rad at 2.09 rad/s nominal → traverse time ≈ 1.5 s.
- Husky DiffDrive wheels are not braked when `/cmd_vel = 0`; reliance on
  wheel friction with ground.
- `mu1 = mu2 = 1.5` set per wheel in URDF Gazebo plugin block.
- Sim `wasd_teleop` / `row_navigator` send `Twist(0,0)` to stop, no
  parking-brake or torque hold.
- TF chain depends on `odom→husky_base_link` from DiffDrive; if the
  base micro-rotates due to arm reaction, `/odom` and `/tf` will report
  it, but planning_scene_monitor + IK may already have used a snapshot
  → known cause of `Invalid Trajectory` errors (mitigated with
  `_wait_for_joint_state` polling in `arm_commander`).
- Real deployment options to consider: outriggers, brake actuation, or
  velocity-scaled arm motion when base motion is detected.

### Q5 — Field condition envelope

> Under what environmental conditions can the deployed system operate,
> and what is the failure mode for each boundary?

**GIVEN:**
- Operating environment: outdoor cotton orchard, gravel/soil aisles,
  row pitch ~3 m, in-row tree pitch median 1.5 m.
- IP rating (manufacturer):
  - Husky A200: **IP44** (splash + small objects)
  - Doosan M1013: **IP54** (dust + water spray); cobot-grade only
  - Hand-E: **IP67** (dust-tight + temp. immersion)
  - RealSense / typical RGB-D cameras: **IP not rated** by default —
    ruggedized housing TBD
- Sim assumes flat ground, no wind, constant lighting; field
  conditions to characterize:
  - Lighting: sunny / cloudy / dusk → CV detector behavior?
  - Wind: shakes bolls and branches → detection noise, dynamic targets
  - Soil: dry / wet / muddy → wheel slip and odom drift
  - Temperature: −10 to +45 °C operating range typical of M1013
  - Dust / pollen on optics: cleaning interval
- No environmental sensors currently in sim (no IMU bias model, no
  weather), to be added if required by analysis.

### Q6 — Localization in field (vs. sim's static TF assumption)

> How does the deployed system know where it is in the orchard, with
> what accuracy, and how does that constrain cluster scout pose
> tolerance?

**GIVEN:**
- Sim TF chain: `world → odom (static, set at launch) → husky_base_link
  (DiffDrive odom integration) → ...`.
- `row_navigator._drive_to(x, y, yaw)` accepts:
  - `pos_tol = 0.20 m`
  - `yaw_tol = 0.10 rad ≈ 5.7°`
  - target `(tree_x, scout_y = 4.85, scout_yaw = 0)`
- Drive feedback is pure `world ← husky_base_link` lookup; no IMU
  fusion, no GPS in sim.
- Real-deployment localization options to evaluate:
  - GPS RTK (cm-level outdoors)
  - Visual-Inertial Odometry from wrist camera + IMU
  - LiDAR SLAM (extra sensor)
  - Wheel odometry alone (drifts with slip, OK for sub-minute drives)
- Required scout pose accuracy is implied by cluster detection
  requirements: cluster center precision after scan is ≤ 5 cm in 3D
  (from dedup radius); the **drive** does not need to be that
  accurate as long as the cluster falls within camera FOV at scout
  pose (~1 m of camera).

### Q7 — Power budget and run time per charge

> What is the energy consumption per cluster cycle and per row, and
> how many cycles fit into a Husky battery charge?

**GIVEN:**
- Husky A200 battery: **24 V × 24 Ah = 576 Wh**, advertised run time
  3 hours under typical driving load.
- M1013 manufacturer power:
  - average draw ~800 W
  - peak ~2.7 kVA
- Hand-E: ~24 V, ~0.5 A nominal.
- Compute target (Jetson Orin Nano / NX): 7–25 W typical, 40 W max.
- Pipeline observed wall-clock timings (sim, used as energy proxy):
  - Cluster scan: 25–35 s — arm motion ~50% of duty
  - Per boll pick: 60–120 s — arm motion ~80% of duty
  - 1 m drive: 4–8 s — drive motors active
  - Full row (3 clusters): ~25 min
- Sim power not modeled directly. Real measurement / analysis needed
  to map sim duty cycle → battery drain.

### Q8 — Harvesting throughput vs. human baseline

> What is the target boll-per-hour throughput, and how does it compare
> against a human picker?

**GIVEN:**
- Observed sim throughput per cluster harvest cycle (with reservoir
  motion failures excluded, robust path):
  - Cluster scan: ~30 s
  - Pick subroutine per boll: ~75 s mean
  - Cluster with 4 ripe bolls: ~30 + 4·75 = **330 s ≈ 5.5 min**
  - Plus 1 m drive between clusters: ~6 s
  - Effective cluster-to-cluster time: ~6 min in nominal flow
- Throughput estimate (sim): **~10 cluster/hr** × ~4 bolls/cluster =
  **~40 bolls/hr ≈ 0.3 kg/hr** assuming 8 g/boll
- Human cotton picker baseline (literature): **25–40 kg/hr**.
- Gap is ~100× — improvements expected from:
  - Multi-boll pick per arm trip (currently 1 boll per round trip)
  - Lower reservoir round-trip cost (currently dominates per-boll time)
  - Faster joint speeds (sim runs at 1.0 scaling but slow trajectories)
- Target throughput is **a design parameter still open**: minimum
  viable to justify deployment, optimistic stretch goal, and what
  software/hardware changes would close which fraction of the gap.

### Q9 — Safety zones and failure modes

> What failure modes can the system encounter, how are they detected,
> and what's the recovery behavior?

**GIVEN:**
- Implemented failure handling today:
  - **Pick fails**: counted as `failed` in summary; loop continues to
    next boll. Re-scan in next `cluster_harvester` iteration may
    re-discover the missed boll (if mock-teleport returned it).
  - **Reservoir reach fail**: warned, carry thread stopped (boll
    frozen at TCP), loop continues. **Currently the boll is left
    suspended mid-air** — recovery TBD.
  - **Cluster scan returns 0 bolls**: `cluster_harvester` exits
    iteration loop → DONE for that cluster.
  - **Drive timeout** (`drive_timeout_s = 90 s`): `_drive_to` returns
    False, `row_navigator` logs FAIL, moves on to next tree.
  - **YAML match fail**: detection has no nearby ground-truth model →
    skipped (sim artifact).
  - **`max_iterations = 3`** safety cap on re-scan loop.
- **Not yet handled** in current pipeline:
  - Branch / unexpected obstacle collision during arm motion
  - Wheel slip / odometry drift detection
  - Battery low / compute thermal throttling
  - E-stop (no physical button modeled in sim)
  - Network drop if compute off-board
- Suggested safety layers for field:
  - Hardware E-stop with brake actuation
  - Force/torque sensor at TCP for contact monitoring
  - Pre-collision check via MoveIt collision matrix on real branches
  - Watchdog timer on `/joint_states` and `/odom`

### Q10 — Compute, latency, and real-time guarantees

> Where does each node run in deployment, what are the latency
> constraints, and is the architecture viable on the target compute?

**GIVEN:**
- Current sim runs all nodes on a single host x86 PC under WSL2 Ubuntu
  22.04, ROS 2 Humble, with `use_sim_time = true`.
- Pipeline node count: 5 perception/harvest nodes + arm_commander +
  gripper_controller + move_group + row_navigator + cluster_scanner +
  cluster_harvester + simple_cluster_harvester ≈ **10 ROS 2 processes**.
- Heaviest CPU consumers observed:
  - MoveGroup / OMPL RRTstar planning: 0.5–3 s per call, single core
  - `cv_boll_detector`: depth + color + contour pipeline ≈ 50 ms /
    detection call (640×480, 10 Hz cap on input)
  - `depth_processor`: < 10 ms per pixel-to-3d call
- Real-time requirements (proposed):
  - Joint state monitor: **< 50 ms** roundtrip for arm safety
  - Camera + detection: **soft real-time**, < 200 ms per detection
  - Motion planning: best effort (1–3 s acceptable)
  - cmd_vel control loop: 10 Hz in row_navigator (deterministic
    Python timer)
- Target deployment compute:
  - Onboard Jetson Orin (single SBC) — viable for current pipeline at
    its current scale
  - Off-board compute via Wi-Fi / 4G — adds latency, needs network QoS
- TF synchronization across nodes was demonstrably fragile during
  multi-stage MoveGroup goals → fix: settle-time + active joint state
  polling in `arm_commander._wait_for_joint_state`.
- State estimation / clock sync:
  - sim uses Gazebo sim time on `/clock` topic
  - real deployment will need NTP/PTP sync if multi-machine

---

## 3. Cross-References

| Concern | File / Service |
|---|---|
| Husky URDF | `src/robot_arm/urdf/husky_robocot.urdf.xacro` |
| M1013 URDF | `src/robot_arm/urdf/m1013_robocot.urdf.xacro` |
| World file | `src/robot_arm/worlds/orchard.world`, `orchard_bolls.world` |
| Bolls inventory | `src/robot_arm/config/orchard_bolls.yaml` |
| Tree positions | `src/robot_arm/config/orchard_tree_positions.yaml` |
| Camera bridge | `src/robot_arm/config/husky_gz_bridge.yaml` |
| Launch — Husky + Gazebo | `src/robot_arm/launch/husky_orchard_demo.launch.py` |
| Launch — MoveIt + arm | `src/robot_arm_moveit_config/launch/moveit.launch.py` |
| Launch — 5-node bundle | `src/orchestrator/launch/harvester_modules.launch.py` |
| arm_commander | `src/robot_arm_moveit_config/robot_arm_moveit_config/arm_commander.py` |
| row_navigator | `src/orchestrator/orchestrator/row_navigator.py` |
| cluster_harvester | `src/orchestrator/orchestrator/cluster_harvester.py` |
| cluster_scanner | `src/orchestrator/orchestrator/cluster_scanner.py` |
| simple_cluster_harvester | `src/orchestrator/orchestrator/simple_cluster_harvester.py` |
| cv_boll_detector | `src/orchestrator/orchestrator/cv_boll_detector.py` |
| depth_processor | `src/orchestrator/orchestrator/depth_processor.py` |

---
*Generated for hand-off to mechanical / dynamic analysis. Section 1 is
self-contained; Section 2 questions should each receive an analytical
answer + safety margin before the report goes to print.*
