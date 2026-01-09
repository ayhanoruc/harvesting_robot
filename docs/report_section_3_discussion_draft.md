# 3. Discussion

This section discusses the key technical decisions and lessons learned during RoboCot development.

## 3.1 Deep Learning vs Classical Computer Vision

Before committing to YOLO, we tested HSV color segmentation as a simpler baseline. The idea was straightforward: cotton bolls are white, so threshold for white pixels in the image. The results were clear but not useful. HSV detected every white object in the frame including the robot's own gripper parts (see Appendix Figure 16). About half of all detections were false positives.

This confirmed what the literature describes: color-only approaches cannot distinguish semantically different objects that happen to share similar colors [3][12]. A white gripper and a white cotton boll have the same color histogram but completely different meaning. YOLO's learned features capture this semantic difference through texture, shape and context that rule-based methods miss entirely.

The takeaway is simple. For harvesting applications where the environment contains visually similar distractors, deep learning is necessary not optional. The extra training effort pays off because the fundamental limitation of classical CV cannot be fixed with better thresholds.

## 3.2 World-Space Clustering vs Frame-Based Tracking

We initially considered video tracking algorithms like ByteTrack to maintain cotton boll IDs across the scan [25][26]. These tools work well for continuous video where objects move smoothly between frames. But our scanning strategy has discrete jumps between 21 positions with large viewpoint changes. Under these conditions ByteTrack kept assigning different IDs to the same physical cluster because the visual appearance changed too much between non-adjacent views.

World-space clustering solved this by operating on 3D coordinates instead of visual features. Detections get back-projected into world space and grouped by spatial proximity. Complete-linkage specifically prevents chain-linking artifacts where distant clusters get incorrectly merged through intermediate detections.

The key advantage is frame-rate independence. Whether detections arrive at 1 Hz or 30 Hz, clustering works on accumulated 3D points rather than temporal sequences. This matches our discrete multi-viewpoint scanning much better than algorithms designed for continuous video.

For future work, combining both approaches could be valuable. Use world-space clustering for the initial panoramic scan, then switch to visual tracking during focused observation of individual clusters where the camera stays relatively stationary.

## 3.3 ROS2 Ecosystem: Learning Curve vs Long-term Benefits

ROS2 Humble with Gazebo Fortress was challenging to learn. Nodes, topics, services, TF2 transforms, launch files, MoveIt2 planning groups, URDF models - there is a lot to understand before anything works. Documentation is scattered and version compatibility between Humble and Fortress caused configuration headaches.

But the investment paid off. When we needed to update the YOLO detector, only that node changed. Everything downstream stayed the same. TF2 handles the camera-to-world coordinate chain automatically once configured.

The skills transfer directly to industry. ROS2 is the standard for robotics development and the concepts (modular architecture, hardware abstraction, simulation-based validation) apply regardless of specific framework. The steep learning curve is an investment that compounds over time.

This experience reinforced that robotics is fundamentally interdisciplinary. Mechanical design, computer vision, ML, control theory and software engineering all affect performance. Weakness in any area creates bottlenecks. This breadth is challenging but also keeps the work engaging with continuous learning opportunities.

## 3.4 ML Model Generalization and Robustness

The YOLO11s model achieved 95% detection accuracy for cotton bolls and 82% for unripe bolls in controlled conditions. These numbers are encouraging but need careful interpretation for real deployment.

The 0.7 confidence threshold was tuned empirically. Lower values caught more bolls but admitted more false detections. Higher values improved precision but missed partially occluded targets. The current value works for the mock field but may need adjustment for different environments.

Dataset coverage remains a limitation. The 375 augmented training images cover rotation, blur and contrast variations, but real fields will have conditions outside this distribution. Heavy dust on the lens, wet cotton after rain, or novel plant configurations could degrade accuracy. The 82% on unripe bolls specifically reflects limited training data for that class.

Future field deployments should include continuous data collection. Failed detections and edge cases get added to the training set for periodic model updates. The modular architecture supports remote model deployment (requirement MA-04) so improvements can be pushed without hardware changes.

## 3.5 Project Scope Evolution and Design Philosophy

The project scope changed significantly during development. Initial requirements were loosely defined and evolved as we understood the technical challenges better. This created pressure to redesign components multiple times.

Our response was to make everything as modular and generic as possible. The orchestrator state machine can be extended with new states. The detection pipeline accepts different YOLO models without code changes. Motion planning uses configurable waypoints. Clustering parameters are exposed as tunable values. This flexibility has costs - more abstraction layers, more configuration, more testing. But when requirements changed, the system adapted without fundamental restructuring.

Some specific problems consumed significant debugging time. A ~20cm systematic positioning error was traced to Gazebo's camera_info message containing inconsistent K and P matrices. The standard ROS function uses the P matrix which had wrong principal point values. Extracting intrinsics directly from K fixed it. The QL-02 repeatability spec (±3mm) was not achieved in simulation (measured σ=7.4mm) due to depth sensor noise and TF timing issues. Hardware with proper calibration should improve this.

The broader lesson is that requirement volatility is normal in research projects. Designing for flexibility from the start reduces the cost of inevitable changes even if it increases initial effort. This applies beyond this specific project to any early-stage development work.

Looking back, the design process developed skills that transfer beyond agricultural robotics. Systems integration, debugging multi-component pipelines, and working with industry tools like ROS2 and MoveIt2 are valuable regardless of application domain. Whether future work continues in agriculture or moves to other robotics areas, RoboCot provided solid foundational experience.
