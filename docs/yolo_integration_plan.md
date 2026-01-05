## YOLO Integration Plan (static feed first)

Goals:
- Run our trained YOLO (`best.pt`) inside the ROS2 graph, providing `/yolo/detect` for the spatial pipeline.
- Start with a static camera feed (single frame or a small set of frames) to verify boxes visually.
- Produce cluster-oriented detections (optionally by merging boll boxes) before moving to live camera streaming.

Reference assets:
- Model: `src/docs/RESEARCH/Cotton-Tracking-YOLO/best.pt` (same copy also in `src/orchestrator/models/best.pt`).
- Working scripts to borrow patterns: `track_webcam_v3.py`, `track_mp4_v2.py` (both use `sticky_tracker.yaml`).
- Static test script and outputs: `test_yolo_static.py`, images under `gazebo_frames/` and `gazebo_frames/output/`.

Phased plan:
1) Baseline static inference outside ROS
   - Use `test_yolo_static.py --conf 0.7 --input gazebo_frames --output gazebo_frames/output` to confirm the weights still produce good boxes on saved Gazebo frames.
   - For cluster-level boxes, reuse the built-in merge (`--cluster --cluster-distance 80`) to see if cluster aggregation looks reasonable.
   - Save a couple of annotated PNGs as evidence for the next step.

2) Package dependencies for ROS node
   - Ensure the runtime environment has `ultralytics`, `opencv-python`, `numpy`, `torch` (GPU if available). If needed, add a minimal `requirements.txt` or a ROS package-level `setup.cfg` dependency list for the new node.
   - Confirm CUDA availability with `check_gpu.py` (already in the folder) to decide GPU vs CPU flags.

3) Implement a ROS2 YOLO service node (static-feed mode first)
   - Location: add a node (e.g., `vision_ml` package or new `yolo_server` file) that serves `/yolo/detect` using `harvester_interfaces/srv/YoloDetect`.
   - Load `best.pt` once; optional tracker config can be skipped for static frames.
   - Input source (static): load a single test image (e.g., `gazebo_frames/ss1.png`) on startup and keep it in memory; the service handler runs inference on that fixed frame for every call.
   - Response mapping: convert detections to `harvester_interfaces/BoundingBox` with `u_min/v_min/u_max/v_max/confidence/label/area`. Use class names from the model; for cluster-first workflow, either:
     - Use class `cotton_boll-cluster` directly if present in the model, or
     - Merge cotton_boll boxes into clusters (reuse logic from `test_yolo_static.py`) and set `label` to `cluster_<n>` or `cotton_cluster`.
   - Keep service name `/yolo/detect` so `SpatialDetectionPipeline` uses it without code changes.

4) Visual validation within the ROS node
   - Add an optional flag/param (e.g., `save_output: true`, `output_path: /tmp/yolo_static.png`) to save the annotated frame each time the service is called.
   - Optionally publish an `Image` topic with the annotated frame for quick RViz/foxglove check.

5) Wire into the existing pipeline with static feed
   - Bring up required nodes: `depth_processor`, `camera_focus` (optional), `spatial_detection_pipeline`, and the new YOLO service node.
   - From `explorer`, run `/explorer/panoramic_scan` with `enable_detection=true`; this will call `/detection/run_at_position` which triggers YOLO.
   - Validate with `/detection/print_results` and `/detection/validate` to compare against `environment_config.yaml`.

6) Move toward live camera after static validation
   - Swap the static frame for a real camera topic or rosbag by adding a subscription to `sensor_msgs/Image` and caching the latest frame; keep the same service interface.
   - Tune confidence/IOU thresholds and (if needed) re-enable `tracker="sticky_tracker.yaml"` for temporal stability once streaming.

Risks / checks:
- TF: both YOLO mock and depth processor need `camera_optical_frame -> world`; ensure it’s available in your sim.
- Class-name alignment: confirm the model’s class labels (from `model.names`) match the labels expected downstream (tracking clusters vs bolls). Adjust the merge/labeling step accordingly.
- Performance: GPU preferred; if CPU-bound, reduce input size or raise `conf` to trim detections.

Quick next actions (for this sprint):
- [ ] Run `test_yolo_static.py` on current frames and save 2–3 annotated outputs.
- [ ] Implement the `/yolo/detect` static-frame ROS node with cluster merge option.
- [ ] Hook into the panoramic scan and verify `/detection/validate` passes with ground-truth positions.
