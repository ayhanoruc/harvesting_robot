# Repo Structure

Git root is `src/`. The four ROS 2 packages under it form the active stack;
everything else is either build output (`build/`, `install/`), runtime
artifacts (`yolo_output/`), or retired code preserved under `_legacy/`.

```
harvesting_ws/
├── src/                            ← git root
│   ├── harvester_interfaces/       Custom msg/srv definitions
│   ├── robot_arm/                  URDF, world, Gazebo bridges, launch
│   ├── robot_arm_moveit_config/    MoveIt 2 config + arm_commander
│   ├── orchestrator/               All harvest + perception + UI nodes
│   ├── yolo_training/              Reproducible YOLO11n training package
│   ├── docs/                       Reports, deployment spec, figures
│   ├── scripts/                    Docker entrypoint, env-verify scripts
│   ├── Dockerfile, Dockerfile.test
│   └── _legacy/                    Retired code (see bottom of file)
├── build/                          colcon build artifacts (gitignored)
├── install/                        colcon install space (gitignored)
└── yolo_output/                    runtime detection PNGs + harvest logs
```

---

## Packages

### `harvester_interfaces`
Custom ROS 2 message and service definitions used by the rest of the stack.

| File | Purpose |
|---|---|
| `msg/BoundingBox.msg`         | 2D bbox + confidence + label |
| `msg/DetectedCluster.msg`     | Cluster centroid + bbox area + scan pose |
| `srv/YoloDetect.srv`          | Trigger detection → list of `BoundingBox` |
| `srv/PixelTo3D.srv`           | (u, v) → world `geometry_msgs/Point` |
| `srv/HarvestBoll.srv`         | Legacy 8-step pick (used by retired `harvest_executor`) |
| `srv/FocusFromPixel.srv`      | Legacy IBVS focus (retired) |
| `srv/FocusFromPosition.srv`   | Legacy IBVS focus (retired) |
| `srv/GetDetectedClusters.srv` | Legacy spatial_detection_pipeline output (retired) |
| `srv/RunDetectionPipeline.srv`| Legacy spatial_detection_pipeline trigger (retired) |

The legacy srv definitions stay registered because the retired nodes in
`_legacy/orchestrator_old/` reference them at import time. They cost nothing
to keep and removing them would couple cleanup to a Python edit pass.

### `robot_arm`
Robot description, Gazebo world, controller config, top-level sim launch.

| Path | Purpose |
|---|---|
| `urdf/m1013_robocot.urdf.xacro`   | Doosan M1013 6-DOF + Hand-E gripper + wrist RGB-D camera, with ros2_control + `gz_ros2_control` plugin |
| `urdf/husky_robocot.urdf.xacro`   | Husky body + 4 wheels + deck + mount + reservoir; composes M1013 via `<xacro:include>` |
| `worlds/`                         | Several Gazebo Ignition Fortress worlds. Default for the demo: `cotton_demo.world` (template-instanced compact orchard, 6 clusters per row). Alternatives kept for variant studies: `orchard.world`, `orchard_bolls.world`, `orchard_pickable.world`, `orchard_pickable_no_trees.world` |
| `models/`                         | SDF model dirs the worlds reference via `model://`. Notably: `branch_variant_A..F_*` (6 photorealistic cotton branches), `cotton_picks` (pickable boll variants), `cotton_cluster`, `cotton_field_ground`, `single_tree` |
| `meshes/`                         | DAE meshes for the M1013 arm, Hand-E gripper, and cluster templates |
| `config/cotton_demo_clusters.yaml`| Ground-truth cluster positions for `cotton_demo.world` (consumed by `row_navigator`) |
| `config/cotton_demo_bolls.yaml`   | Ground-truth boll inventory (consumed by `simple_cluster_harvester` and `cluster_harvester` for detection→ID matching) |
| `config/orchard_*.yaml`           | Same role for legacy orchard worlds |
| `config/branch_variant_targets.yaml`, `cotton_cluster_sockets.yaml`, `cotton_targets*.yaml` | Per-variant boll socket coordinates inside each branch model (used by `scripts/generate_cotton_demo.py` to populate the world + boll yamls) |
| `config/husky_gz_bridge.yaml`     | `ros_gz_bridge` topic config: `/cmd_vel`, `/odom`, `/clock`, `/tf`, `/camera/{color,depth}/{image,camera_info}` |
| `config/environment_config.yaml`  | Legacy named targets file. Still loaded by `arm_commander` as a fallback for `/go_to_named` — active pipeline calls `/go_to_pose` (xyz) instead so the contents are dead but the file stays to avoid a startup warning |
| `yaml/controllers.yaml`           | `ros2_control` controllers: `joint_state_broadcaster`, `arm_controller` (6 joints), `gripper_controller` (`hande_left_finger_joint`) |
| `launch/husky_orchard_demo.launch.py` | Top-level sim launch. Spawns Gazebo, the Husky+arm URDF, the GZ↔ROS bridges, and the three controller spawners (sequenced) |
| `robot_arm/landmark_publisher.py` | Publishes static TF + collision objects from `environment_config.yaml`. Currently dormant (collision objects yaml entry is `{}`) but still installed by `CMakeLists.txt` |
| `scripts/generate_*.py`           | Generators that emit `*.world` + `*.yaml` from template variants. Used at content-authoring time, not at runtime |

### `yolo_training`
Not a ROS package — the reproducible training pipeline for the YOLO11n
weights consumed by `real_yolo_detector`. Owned by the project's ML side
(Deniz). Self-contained: a Python venv + the entrypoints + the dataset
config + the metrics from the most recent run.

| Path | Purpose |
|---|---|
| `weights/best.pt`             | YOLO11n weights the orchestrator loads (identical bytes as `orchestrator/models/best.pt`) |
| `weights/yolo11n.pt`          | Base model used for fine-tuning, kept for reproducibility |
| `configs/data.yaml`           | Roboflow dataset config — `deniz-drin5/cotton-boll-and-cluster v5`, 2 classes |
| `configs/training_args.yaml`  | Frozen Ultralytics args from the 80-epoch run (imgsz 512, batch 4, seed 42) |
| `metrics/results.csv` + PNGs  | Loss / mAP / precision / recall curves + confusion matrix |
| `src/train.py`                | Training entrypoint |
| `src/validate.py`             | Val/test metrics entrypoint |
| `src/predict.py`              | Inference on image / folder / video / webcam |
| `src/summarize_results.py`    | Quick summary of `metrics/results.csv` |
| `src/exp/`                    | Older webcam + BoT-SORT tracking experiments, kept for reference |
| `requirements.txt`            | Standalone deps (ultralytics + opencv + numpy etc.) for the training venv |
| `README.md`                   | Full walkthrough: setup, dataset, train, validate, infer |

To retrain or re-validate, see the package's own
[yolo_training/README.md](yolo_training/README.md). The active ROS demo
does not require this folder — `best.pt` is already installed under
`orchestrator/models/`.

### `robot_arm_moveit_config`
MoveIt 2 motion planning config + the arm commander node.

| File | Purpose |
|---|---|
| `config/robot_arm.srdf`           | Planning groups (`arm`, `gripper`), named poses, collision matrix |
| `config/kinematics.yaml`          | KDL IK solver, 0.5 s timeout, 20 attempts |
| `config/ompl_planning.yaml`       | OMPL planner config — `arm` uses RRTstar by default |
| `config/joint_limits.yaml`        | Per-joint velocity / acceleration caps |
| `config/moveit_controllers.yaml`  | Maps MoveIt actions to the ros2_control controllers |
| `config/ros2_controllers.yaml`    | Standalone copy of controller config (used by RViz panel) |
| `launch/moveit.launch.py`         | Brings up `move_group`, RViz, `arm_commander`, and `gripper_controller_node` |
| `robot_arm_moveit_config/arm_commander.py` | IK + joint-goal services: `/go_to_pose`, `/go_to_named`, `/go_to_reservoir`, `/go_home_view`. Wraps MoveIt's `MoveGroup` action for HOME and uses direct `JointTrajectory` publishes for pipeline moves to avoid planning_scene staleness |

### `orchestrator`
All pipeline logic above the arm: perception, scan, pick, drive, UI.

The active pipeline is layered. Each layer is one or more ROS 2 nodes; layer N calls services exposed by layer N−1.

| Layer | Node | Service / role |
|---|---|---|
| 3 | `row_navigator`              | `/row_nav/run` — iterate a route of cluster ids; drive Husky to each scout pose (closed-loop `cmd_vel` + TF feedback + Gazebo-truth odom recalibration); call `/cluster_harvester/run` at each stop |
| 2 | `cluster_harvester`          | `/cluster_harvester/run` — scan → match detections to YAML boll ids → sort by reach → trigger pick batch → re-scan until empty |
| 1 | `cluster_scanner`            | `/cluster_scan/run` — pan/tilt arm sweep, per-pose `/yolo/detect` + `/depth_processor/pixel_to_3d`, dedup in 3D, gap-rule cluster bounding, write JSON + top-down PNG |
| 1 | `simple_cluster_harvester`   | `/simple_harvest/start` — per-boll pick: `/go_to_pose` → mock close → `set_pose` teleport boll to TCP → `/go_to_reservoir` → mock open → teleport into bin. Carry threads keep already-dropped bolls glued to the reservoir as Husky moves |
| 0 | `real_yolo_detector`         | `/yolo/detect`, `/yolo/detect_clusters` — YOLO11 inference on `/camera/color/image_raw`, saves annotated PNGs |
| 0 | `cv_boll_detector`           | Same two services, physics-based classical CV alternative. Plug-and-play swap (run instead of `real_yolo_detector`, not alongside) |
| 0 | `mock_yolo_detector`         | Same two services, projects YAML ground truth into pixel space. For TF / depth sanity checks without YOLO |
| 0 | `depth_processor`            | `/depth_processor/pixel_to_3d` — back-projects pixel via K matrix, transforms `camera_optical_frame` → `world`. 5×5 windowed median fixes sparse-mesh holes |
| 0 | `gripper_controller`         | `/gripper/open`, `/gripper/close` — publish to `/gripper_controller/commands`, poll `/joint_states` for convergence |

Tooling and UI:

| Node | Purpose |
|---|---|
| `control_panel`   | PyQt5 GUI. Auto-launches the sim on startup. One-click `Start the arm` (MoveIt) and `Start the car engine` (`row_navigator` + `harvester_modules.launch.py`). WASD base teleop + on-screen camera-arm teleop. Live camera feed + latest detection image + telemetry tiles |
| `wasd_teleop`     | Terminal-side `cmd_vel` publisher for the Husky base |
| `arm_teleop`      | Terminal-side wrist-camera teleop (publishes `JointTrajectory` directly, bypasses MoveIt) |
| `sim_helpers`     | `/sim/spawn_at_cluster` — teleport the Husky model in Gazebo to a scout pose. Useful for perception tests that don't need navigation |

| Launch file | What it brings up |
|---|---|
| `launch/harvester_modules.launch.py` | The five pipeline nodes you keep running together: `real_yolo_detector`, `depth_processor`, `cluster_scanner`, `simple_cluster_harvester`, `cluster_harvester` |

---

## Running the active demo

Three terminals (or one click in `control_panel`):

```
ros2 launch robot_arm husky_orchard_demo.launch.py
ros2 launch robot_arm_moveit_config moveit.launch.py
ros2 launch orchestrator harvester_modules.launch.py
ros2 run    orchestrator row_navigator
```

Then trigger the row:

```
ros2 service call /row_nav/run std_srvs/srv/Trigger '{}'
```

Or trigger just one cluster scan:

```
ros2 service call /cluster_scan/run std_srvs/srv/Trigger '{}'
```

The Control Panel wraps all of this:

```
ros2 run orchestrator control_panel
```

---

## Plug-and-play extension points

The pipeline is split along service boundaries on purpose — each layer
talks to the one below it through a named service with a fixed schema.
Swapping a layer means matching that schema; nothing else has to move.

### Add a new detector
Implement a node that advertises:
- `/yolo/detect` (`harvester_interfaces/srv/YoloDetect`) — return raw boll bboxes
- `/yolo/detect_clusters` (`harvester_interfaces/srv/YoloDetect`) — return merged cluster bboxes

It should subscribe to `/camera/color/image_raw`. The three existing
detectors (`real_yolo_detector`, `cv_boll_detector`, `mock_yolo_detector`)
are interchangeable references. Drop your node into
`orchestrator/orchestrator/`, add an entry in `setup.py`, edit
`harvester_modules.launch.py` to launch yours instead of
`real_yolo_detector`. No other node changes required.

### Add a new gripper
Implement a node that advertises:
- `/gripper/open`  (`std_srvs/Trigger`)
- `/gripper/close` (`std_srvs/Trigger`)

The mock pick path teleports the boll to TCP regardless of grasp
physics, so the gripper services only need to (a) actuate the URDF
joint(s) for visual feedback and (b) return success when the joint
converges. If you target real hardware, the same node should drive the
hardware driver (e.g. `pymodbus` for a Robotiq Hand-E) instead of
publishing to `/gripper_controller/commands`.

### Add a new world
Drop the `.world` file under `robot_arm/worlds/` and a matching boll
inventory yaml under `robot_arm/config/` (see
`cotton_demo_bolls.yaml` for the schema: a top-level `items:` list with
`id`, `tree_id`, `x`, `y`, `z`, `model` keys). The generator scripts in
`robot_arm/scripts/` (`generate_cotton_demo.py`,
`generate_pickable_orchard.py`) emit both files from a template.

Then either run with `world:=your_world.world` on the existing launch,
or set the relevant `*_yaml` parameter on the consumers
(`cluster_harvester`, `simple_cluster_harvester`, `row_navigator`).

### Add a new world content model
Drop the model directory under `robot_arm/models/`. Reference it from
your world file via `model://<dirname>`. The Gazebo resource path is
extended in `husky_orchard_demo.launch.py` to include
`robot_arm/models/` and `robot_arm/models/cotton_picks/`, so models in
those locations resolve automatically.

### Add a new pipeline stage
The existing four layers are not exhaustive — you can wedge a new
service-call step into any of them. Two clean insertion points:

- **Pre-pick validation** — call your service from
  `cluster_harvester._on_run` between `_match_detections_to_ids` and
  `_trigger_harvest`. Return a filtered id list.
- **Per-boll post-grasp check** — call your service from
  `simple_cluster_harvester` inside the per-boll loop, after the mock
  close and before the carry-to-reservoir leg.

In both cases your node only needs to expose a `std_srvs/Trigger` (or
a custom srv with a position field) and the upstream node will need a
five-line edit to call it.

### Add a new robot base or arm
URDF and ros2_control are decoupled from the pipeline. Replace the URDF
referenced by `husky_orchard_demo.launch.py`, update
`yaml/controllers.yaml` to match the new joints, and update the SRDF +
kinematics yaml in `robot_arm_moveit_config/config/` for MoveIt. The
orchestrator nodes reference frames (`tcp`, `base_0`, `husky_base_link`)
and joint names (`joint1..joint6`, `hande_left_finger_joint`) directly,
so a new arm with different naming will need a search-and-replace pass
across the nodes.

---

## `_legacy/`

Code retired but kept on disk for reference. Nothing in `_legacy/` is
imported, launched, or installed by the active build. Contents:

- `4dof_era/` — From the pre-Husky M1013-on-a-table phase: `mybot.urdf.xacro`, the four old launch files (`bot.launch.py`, `bot.launch_old.py`, `orchard_test.launch.py`, `husky_test.launch.py`, `husky_autonomous.launch.py`), the old GZ bridge config, the original CMakeLists, the original controllers yaml, and the `config_old/` MoveIt directory
- `orchestrator_old/` — The pre-`row_navigator` state-machine pipeline: `main.py` (state machine), `harvest_executor.py` (8-step pick), `harvest_orchestrator.py` (ground-truth autonomous demo), `explorer.py` (panoramic scan, superseded by `cluster_scanner`), `camera_focus.py` (IBVS focus, superseded by world-space dedup), `spatial_detection_pipeline.py` (detection coordinator), and `harvest_pipeline.launch.py` (the launch that wired them all together). The matching entry points in `orchestrator/setup.py` are commented out; uncomment them and copy a file back if you need to revive one
- `remotion-dashboard/` — The TypeScript/Remotion video dashboard, superseded by `control_panel.py`
- `vision_ml/`, `robotic_actor/`, `logger_node/`, `example_arm_*` — Older packages from earlier project phases, never integrated into the current pipeline

The retired nodes still reference legacy srv types
(`HarvestBoll.srv`, `FocusFromPixel.srv`, `FocusFromPosition.srv`,
`GetDetectedClusters.srv`, `RunDetectionPipeline.srv`). Those defs are
kept in `harvester_interfaces/srv/` so the legacy files remain
parseable in place; the active pipeline doesn't use them.
