"C:\Users\ayhan\harvesting_ws"
wsl -d Ubuntu-22.04
cd /mnt/c/Users/ayhan/harvesting_ws

colcon build --packages-select robot_arm robot_arm_moveit_config

source install/setup.bash

export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch robot_arm bot.launch.py

ros2 launch robot_arm_moveit_config moveit.launch.py

ros2 run rqt_image_view rqt_image_view

ros2 param set /arm_commander target_name cluster_1

ros2 service call /go_to_named std_srvs/srv/SetBool "{data: true}"

colcon build --packages-select robot_arm robot_arm_moveit_config orchestrator

ros2 run orchestrator explorer

ros2 service call /explorer/start_scan std_srvs/srv/Trigger "{}"

ros2 service call /explorer/panoramic_scan std_srvs/srv/Trigger "{}"


ros2 run orchestrator wasd_teleop


# Build with detection pipeline
colcon build --packages-select harvester_interfaces orchestrator robot_arm robot_arm_moveit_config


# Run panoramic scan with detection (enable_detection=true by default)
ros2 service call /explorer/panoramic_scan std_srvs/srv/Trigger "{}"

# Disable detection for faster scanning
ros2 param set /explorer enable_detection false
ros2 service call /explorer/panoramic_scan std_srvs/srv/Trigger "{}"

# Manual detection commands
ros2 service call /yolo/detect harvester_interfaces/srv/YoloDetect "{}"
ros2 service call /detection/run_at_position std_srvs/srv/Trigger "{}"
ros2 service call /detection/validate std_srvs/srv/Trigger "{}"
ros2 service call /detection/print_results std_srvs/srv/Trigger "{}"
ros2 service call /detection/clear std_srvs/srv/Trigger "{}"

 
  ros2 run orchestrator camera_focus &
  ros2 run orchestrator real_yolo_detector &
  ros2 run orchestrator depth_processor &
  ros2 run orchestrator spatial_detection_pipeline


  # Terminal 2: Build & run
  cd /mnt/c/Users/ayhan/harvesting_ws
  colcon build --packages-select orchestrator
  source install/setup.bash

  # Run YOLO detector
  ros2 run orchestrator real_yolo_detector


  mkdir -p /mnt/c/Users/ayhan/harvesting_ws/yolo_output && cp /tmp/yolo_detections/* /mnt/c/Users/ayhan/harvesting_ws/yolo_output/


  ros2 service call /detection/run_at_position std_srvs/srv/Trigger "{}"


   ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller


---

ros2 launch robot_arm husky_autonomous.launch.py

ros2 launch robot_arm_moveit_config moveit.launch.py

ros2 run orchestrator simple_cluster_harvester

ros2 run orchestrator harvest_orchestrator

ros2 service call /harvest/start std_srvs/srv/Trigger '{}'


---
# === Per-cluster perception test (camera + YOLO; static, no navigation) ===
# Spawn directly in front of tree_000 via launch args (or hop later with sim_helpers).

colcon build --packages-select orchestrator
source install/setup.bash

ros2 launch robot_arm husky_orchard_demo.launch.py spawn_x:=15.8 spawn_y:=4.85 spawn_yaw:=-1.5708
ros2 launch robot_arm_moveit_config moveit.launch.py
ros2 run orchestrator real_yolo_detector
ros2 run orchestrator sim_helpers

# (optional) live camera view
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw

# YOLO prediction (annotated PNG auto-saved to /mnt/c/Users/ayhan/harvesting_ws/yolo_output)
ros2 service call /yolo/detect          harvester_interfaces/srv/YoloDetect '{}'
ros2 service call /yolo/detect_clusters harvester_interfaces/srv/YoloDetect '{}'

# Teleport Husky to a different cluster (cluster_id is a parameter)
ros2 param set /sim_helpers cluster_id tree_005
ros2 service call /sim/spawn_at_cluster std_srvs/srv/Trigger '{}'
# then re-run /yolo/detect ...

# Manual wrist-camera steering (joint-space WASD)
ros2 run orchestrator arm_teleop
# a/d = pan, w/s = tilt, e/q = higher/lower, r/f = extend/retract, z/x = roll
# space = stop, h = HOME, +/- = speed scale, CTRL-C = quit

# Classical-CV boll detector (sim-targeted; drop-in for real_yolo_detector)
# Detects bright white spheres → BoundingBox; uses depth for sanity filter.
# Stop real_yolo_detector first (services collide), then:
ros2 run orchestrator cv_boll_detector
ros2 service call /yolo/detect          harvester_interfaces/srv/YoloDetect '{}'
ros2 service call /yolo/detect_clusters harvester_interfaces/srv/YoloDetect '{}'

# Tune HSV at runtime if needed (lower V or raise S_max for more permissive mask):
ros2 param set /cv_boll_detector hsv_v_min 150
ros2 param set /cv_boll_detector hsv_s_max 90


---
# === Cluster scanner — static arm sweep + boll consolidation + cluster bbox ===
# Husky stays put at cluster scout pose; arm sweeps pan/tilt grid.
# Saves cluster JSON + top-down PNG to yolo_output/.

colcon build --packages-select orchestrator
source install/setup.bash

# Spawn at cluster_1 (tree_000) scout
ros2 launch robot_arm husky_orchard_demo.launch.py spawn_x:=15.8 spawn_y:=4.85 spawn_yaw:=-1.5708
ros2 launch robot_arm_moveit_config moveit.launch.py
ros2 run orchestrator cv_boll_detector
ros2 run orchestrator depth_processor      # ← needed for 3D back-projection (world frame)
ros2 run orchestrator cluster_scanner

# Trigger the full scan (sweep + detect + dedup + cluster bbox + save JSON/PNG)
ros2 service call /cluster_scan/run std_srvs/srv/Trigger '{}'

# Saved files (timestamped):
#   /mnt/c/Users/ayhan/harvesting_ws/yolo_output/cluster_scan_<ts>.json   ← full data + bbox
#   /mnt/c/Users/ayhan/harvesting_ws/yolo_output/cluster_topdown_<ts>.png ← XY map w/ bbox

# Runtime tuning if needed:
ros2 param set /cluster_scanner pan_angles_deg "[-30.0, -15.0, 0.0, 15.0, 30.0]"
ros2 param set /cluster_scanner tilt_angles_deg "[-15.0, 0.0, 15.0]"
ros2 param set /cluster_scanner gap_threshold_m 0.8   # tighter cluster boundary
ros2 param set /cluster_scanner cluster_axis y        # if row aligned along Y


---
# === Full single-cluster pipeline (scan → match → pick → re-scan → DONE) ===
# Reuses simple_cluster_harvester for picking (robust heuristic: base rotation
# + carry-during-pick + reservoir-carry). Loops until 0 cluster bolls.

colcon build --packages-select orchestrator
source install/setup.bash

# Same 4 prerequisite terminals as cluster_scanner section above
ros2 launch robot_arm husky_orchard_demo.launch.py spawn_x:=15.8 spawn_y:=4.85 spawn_yaw:=-1.5708
ros2 launch robot_arm_moveit_config moveit.launch.py
ros2 run orchestrator cv_boll_detector
ros2 run orchestrator depth_processor
ros2 run orchestrator cluster_scanner
ros2 run orchestrator simple_cluster_harvester
ros2 run orchestrator cluster_harvester   # ← top-level pipeline node

# Trigger the full cycle (scan + pick all + re-scan + DONE)
ros2 service call /cluster_harvester/run std_srvs/srv/Trigger '{}'

# Tuning
ros2 param set /cluster_harvester max_iterations 5     # more re-scan tries
ros2 param set /cluster_harvester match_radius_m 0.08  # looser detection→YAML match

---
---
# 8 terminal sırayla
ros2 launch robot_arm husky_orchard_demo.launch.py spawn_x:=15.8 spawn_y:=4.85 spawn_yaw:=-1.5708
ros2 launch robot_arm_moveit_config moveit.launch.py
ros2 run orchestrator cv_boll_detector
ros2 run orchestrator depth_processor
ros2 run orchestrator cluster_scanner
ros2 run orchestrator simple_cluster_harvester
ros2 run orchestrator cluster_harvester
ros2 run rqt_image_view rqt_image_view

# Trigger
ros2 service call /cluster_harvester/run std_srvs/srv/Trigger '{}'


---
# === Row navigator — full-row pipeline (drive + harvest + drive + harvest ...) ===
# Layer 3: walks a fixed list of tree_ids, driving Husky to each scout pose
# with closed-loop cmd_vel + TF feedback, then triggers cluster_harvester.

colcon build --packages-select orchestrator
source install/setup.bash

# Same prereqs + row_navigator on top
ros2 launch robot_arm husky_orchard_demo.launch.py spawn_x:=15.8 spawn_y:=4.85 spawn_yaw:=-1.5708
ros2 launch robot_arm_moveit_config moveit.launch.py
ros2 run orchestrator cv_boll_detector
ros2 run orchestrator depth_processor
ros2 run orchestrator cluster_scanner
ros2 run orchestrator simple_cluster_harvester
ros2 run orchestrator cluster_harvester
ros2 run orchestrator row_navigator     # ← top-level row pipeline

# Trigger full row run (tree_000 → tree_001 → tree_002 + harvest each)
ros2 service call /row_nav/run std_srvs/srv/Trigger '{}'

# Override route at runtime:
ros2 param set /row_navigator route "['tree_000', 'tree_001', 'tree_002', 'tree_003']"


---
# === Compact launch — bundle the 5 harvester nodes into one terminal ===
# Saves you from juggling 5 individual `ros2 run` terminals.

ros2 launch robot_arm husky_orchard_demo.launch.py spawn_x:=15.8 spawn_y:=4.85 spawn_yaw:=-1.5708
ros2 launch robot_arm_moveit_config moveit.launch.py
ros2 launch orchestrator harvester_modules.launch.py   # ← cv + depth + cluster_scanner + simple_harvester + cluster_harvester
ros2 run orchestrator row_navigator                    # ← top-level (separate so you can restart it alone)

# Trigger
ros2 service call /row_nav/run std_srvs/srv/Trigger '{}'