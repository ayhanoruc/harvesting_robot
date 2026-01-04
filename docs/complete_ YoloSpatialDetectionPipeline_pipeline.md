  ---
  Complete System Review

  Your Original Request Recap:

  1. Detection Flow: RGB + depth → YOLO bbox → center pixel → focus (2 iters) → get 3D position
  2. Validation: Compare detected 3D with ground truth (5cm tolerance)
  3. Transform Chain: pixel (u,v) + depth → camera_frame → world
  4. At each panoramic position: Run detection sub-loop with 2 focus iterations
  5. Cluster tracking: Keep all detections, select best by highest bbox area
  6. Z offset: Correct for mesh origin vs visible center

  ---
  Files Created/Modified:

  | File                                              | Type     | Purpose                                                         |
  |---------------------------------------------------|----------|-----------------------------------------------------------------|
  | harvester_interfaces/msg/BoundingBox.msg          | NEW      | YOLO bbox: u_min, v_min, u_max, v_max, confidence, label, area  |
  | harvester_interfaces/msg/DetectedCluster.msg      | NEW      | Cluster with 3D position, confidence, bbox_area, num_detections |
  | harvester_interfaces/srv/YoloDetect.srv           | NEW      | Returns array of BoundingBox                                    |
  | harvester_interfaces/srv/RunDetectionPipeline.srv | NEW      | Trigger detection at position                                   |
  | orchestrator/mock_yolo_detector.py                | NEW      | Mock YOLO - projects known positions to pixels                  |
  | orchestrator/spatial_detection_pipeline.py        | NEW      | Main detection pipeline coordinator                             |
  | orchestrator/depth_processor.py                   | FIXED    | Topic names: /camera/depth/...                                  |
  | orchestrator/explorer.py                          | MODIFIED | Integrated detection calls                                      |
  | orchestrator/setup.py                             | MODIFIED | Added entry points                                              |
  | harvester_interfaces/CMakeLists.txt               | MODIFIED | Added msg/srv                                                   |

  ---
  Step-by-Step Flow:

  ┌─────────────────────────────────────────────────────────────────────┐
  │                         STARTUP SEQUENCE                             │
  └─────────────────────────────────────────────────────────────────────┘

  Terminal 1: ros2 launch robot_arm bot.launch.py
      └── Gazebo + Robot + Camera + Bridges

  Terminal 2: ros2 launch robot_arm_moveit_config moveit.launch.py
      └── MoveIt2 + arm_commander (goes to HOME)

  Terminal 3: ros2 run orchestrator explorer
      └── Panoramic scan service ready

  Terminal 4: ros2 run orchestrator mock_yolo_detector
      └── Subscribes to /camera/color/camera_info
      └── Loads ground truth from environment_config.yaml
      └── Provides /yolo/detect service

  Terminal 5: ros2 run orchestrator depth_processor
      └── Subscribes to /camera/depth/camera_info
      └── Subscribes to /camera/depth/image_raw
      └── Provides /depth_processor/pixel_to_3d service

  Terminal 6: ros2 run orchestrator spatial_detection_pipeline
      └── Subscribes to /detection/current_position
      └── Provides /detection/run_at_position service
      └── Provides /detection/validate service

  Terminal 7: ros2 service call /explorer/panoramic_scan std_srvs/srv/Trigger "{}"
      └── Triggers the scan!

  ┌─────────────────────────────────────────────────────────────────────┐
  │                      PANORAMIC SCAN EXECUTION                        │
  └─────────────────────────────────────────────────────────────────────┘

  For each of 21 positions (7 cols × 3 rows):

  1. MOVE TO POSITION
     explorer → /arm_controller/joint_trajectory
     └── Publishes JointTrajectory with target joints
     └── Waits move_duration + 0.5s buffer

  2. PUBLISH POSITION NAME  
     explorer → /detection/current_position (String)
     └── e.g., "middle_far_left"

     spatial_detection_pipeline receives this via subscriber
     └── Updates self.current_scan_position

  3. PAUSE FOR CAPTURE
     explorer sleeps for pause_duration (1.0s)

  4. CALL DETECTION PIPELINE
     explorer → /detection/run_at_position (Trigger)
     
     ┌─────────────────────────────────────────────────────────────────┐
     │               DETECTION PIPELINE (per position)                  │
     └─────────────────────────────────────────────────────────────────┘
     
     4a. YOLO DETECT
         spatial_detection_pipeline → /yolo/detect
         
         mock_yolo_detector:
         ├── Gets camera pose via TF: camera_optical_frame → world
         ├── For each known cluster (cluster_1, cluster_2, cluster_3):
         │   ├── Transform 3D position to camera frame
         │   ├── Project to pixel: (u, v) = K × [X/Z, Y/Z, 1]
         │   ├── If in image bounds → create BoundingBox
         │   └── Add noise (±5 pixels) for realism
         └── Returns array of BoundingBox

     4b. FOR EACH DETECTION (bbox):

         Initial: center_u = (u_min + u_max) / 2
                  center_v = (v_min + v_max) / 2

         ┌─────────────────────────────────────────────────────────────┐
         │                FOCUS LOOP (2 iterations)                     │
         └─────────────────────────────────────────────────────────────┘

         For i = 1 to 2:

           i. CALL FOCUS SERVICE
              spatial_detection_pipeline → /camera_focus/center_on_pixel

              camera_focus:
              ├── Read current joint positions from /joint_states
              ├── Compute pixel error: error = pixel - image_center
              ├── Compute joint adjustments:
              │   ├── hip_delta = -gain_hip × error_u  (horizontal)
              │   ├── shoulder_delta = gain_shoulder × error_v  (vertical)
              │   └── elbow_delta = -gain_elbow × error_v
              ├── Clamp to max_adjustment (0.3 rad)
              ├── Send to MoveIt via move_action
              └── Wait for execution complete

           ii. WAIT 0.5s for arm to settle

           iii. RE-DETECT YOLO
                spatial_detection_pipeline → /yolo/detect
                └── Get new bounding box (larger area = better centered)
                └── Update center_u, center_v, bbox

         After 2 focus iterations, we have final_u, final_v, final_bbox

         ┌─────────────────────────────────────────────────────────────┐
         │                    GET 3D POSITION                           │
         └─────────────────────────────────────────────────────────────┘

         spatial_detection_pipeline → /depth_processor/pixel_to_3d

         depth_processor:
         ├── Get depth at pixel (final_u, final_v)
         ├── Back-project to camera frame:
         │   X_cam = (u - cx) × depth / fx
         │   Y_cam = (v - cy) × depth / fy
         │   Z_cam = depth
         ├── Lookup TF: camera_optical_frame → world
         ├── Transform point to world frame
         └── Return world position (x, y, z)

         ┌─────────────────────────────────────────────────────────────┐
         │                  Z OFFSET CORRECTION                         │
         └─────────────────────────────────────────────────────────────┘

         position_3d.z -= z_offset  (0.03m)

         Why? Ground truth = mesh origin (bottom of cotton)
              Detection = visible center (higher than bottom)
              So: detected_z - 0.03 ≈ ground_truth_z

         ┌─────────────────────────────────────────────────────────────┐
         │                   STORE DETECTION                            │
         └─────────────────────────────────────────────────────────────┘

         Detection {
             cluster_label: "cluster_1"
             position_3d: [0.75, 0.45, 0.46]
             confidence: 0.87
             bbox_area: 4200  ← key for selecting best
             scan_position: "middle_far_left"
             pixel_center: (320, 240)
         }

         Add to tracked_clusters[label].detections[]

  5. MOVE TO NEXT POSITION
     └── Repeat for all 21 positions

  ┌─────────────────────────────────────────────────────────────────────┐
  │                     AFTER SCAN COMPLETE                              │
  └─────────────────────────────────────────────────────────────────────┘

  1. RETURN TO HOME
     explorer → /arm_controller/joint_trajectory
     └── joints = [0.0, -1.3, 1.5, 0.0]

  2. VALIDATE AGAINST GROUND TRUTH
     explorer → /detection/validate

     spatial_detection_pipeline:
     ├── For each ground truth cluster (cluster_1, cluster_2, cluster_3):
     │   ├── Get tracked_clusters[label].best_detection
     │   │   └── Selected by MAX(bbox_area) ← your requirement!
     │   ├── detected_pos = best_detection.position_3d
     │   ├── error = ||detected_pos - ground_truth_pos||
     │   └── PASS if error ≤ 0.05m (5cm tolerance)
     └── Print results

  ┌─────────────────────────────────────────────────────────────────────┐
  │                       EXPECTED OUTPUT                                │
  └─────────────────────────────────────────────────────────────────────┘

  ============================================================
  VALIDATION RESULTS
  ============================================================
  cluster_1: error=0.032m [PASS] (detected=[0.75, 0.45, 0.46], gt=[0.75, 0.45, 0.46])
  cluster_2: error=0.028m [PASS] (detected=[0.85, 0.01, 0.52], gt=[0.85, 0.00, 0.52])
  cluster_3: error=0.041m [PASS] (detected=[0.74, -0.44, 0.42], gt=[0.75, -0.45, 0.42])
  Average error: 0.034m
  Valid: 3/3 (tolerance: 0.05m)

  ---
  Key Parameters:

  | Parameter            | Value              | Location                      |
  |----------------------|--------------------|-------------------------------|
  | Focus iterations     | 2                  | spatial_detection_pipeline.py |
  | Validation tolerance | 0.05m (5cm)        | spatial_detection_pipeline.py |
  | Z offset correction  | 0.03m (3cm)        | spatial_detection_pipeline.py |
  | Panoramic grid       | 7×3 = 21 positions | explorer.py                   |
  | Pause duration       | 1.0s               | explorer.py                   |
  | Move duration        | 1.5s               | explorer.py                   |
