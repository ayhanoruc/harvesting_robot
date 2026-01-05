 Session Summary

  1. Integrated real YOLO into ROS2 with /yolo/detect (raw bolls) and /yolo/detect_clusters (pixel-merged) endpoints, replacing mock detector. Model detects cotton_boll class with 0.7+ confidence.
  2. Spatial Detection Pipeline flow: YOLO detects boll bboxes → extract pixel centers → (optional) camera focus iterations → depth lookup → back-project to 3D world coordinates → cluster in world-space.
  3. World-space clustering uses complete-linkage algorithm: a boll joins a cluster only if it's within merge_radius of ALL existing members (not just one). This prevents chain-linking artifacts that single-linkage causes.
  4. Auto-calculated merge_radius: 25% of minimum X,Y distance between ground truth clusters (0.485m × 0.25 = 0.121m). This ensures bolls on the same plant group together while keeping separate plants apart.
  5. Two clustering approaches exist: pixel-space in real_yolo_detector (quick heuristic, unreliable at varying distances) vs world-space in spatial_detection_pipeline (uses depth + TF, reliable in meters).
  6. Back-projection math: X = (u - cx) * depth / fx, Y = (v - cy) * depth / fy, Z = depth. Then TF transforms from camera_optical_frame to world frame.
  7. Discovered critical camera intrinsics bug: Gazebo's P matrix had wrong principal point (cx=160, cy=120 instead of 320, 240), causing ~20cm systematic error in all 3D projections. PinholeCameraModel uses P matrix, not K matrix.
  8. Fixed by bypassing PinholeCameraModel.projectPixelTo3dRay() and using K matrix directly for back-projection in depth_processor.py. Error reduced from ~20cm to ~1-2cm.      
  9. Added <lens><intrinsics> to URDF to set correct K matrix values (fx=277, fy=277, cx=320, cy=240), but Gazebo still generates wrong P matrix - hence the code workaround.    
  10. Image saving added to both nodes: detect_*.png shows raw YOLO boxes, spatial_*.png shows merged cluster bboxes with 3D positions overlaid. Output to yolo_output/ folder.  
  11. Key lesson: Always verify both K and P matrices in camera_info - Gazebo may set them inconsistently. When debugging 3D projection errors, trace the full pipeline: pixel → camera frame → world frame.