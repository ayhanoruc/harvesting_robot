# RoboCot - Autonomous Cotton Harvesting System
## ME 429 Design Report - Sections 2.2 & 2.3

---

# 2.2 Overview of Possible Solutions

Autonomous cotton harvesting requires the integration of multiple subsystems spanning perception, motion planning and mechanical design. Rather than evaluating monolithic system configurations, this section adopts a component-based approach where each design decision point is analyzed independently. Criteria weights are derived from the Binary Dominance Matrix established in Section 2.1, with each decision matrix applying weights proportional to the relevant PDS criteria. The selected components are then integrated into the final system configuration.

## 2.2.1 Solution Space Overview

The design of an autonomous cotton picking system involves six key decision points spanning software architecture, perception, control and mechanical domains. Table 1 summarizes these decision points, their requirements derived from the product design specifications (Section 2.1) and their impact on overall system performance.

**Table 1. Solution Space: Design Decision Points**

| Decision Point | Requirement Reference | Impact on System |
|----------------|----------------------|------------------|
| Camera Placement | QL-03 (90% detection), SR-02 (mock field coverage) | Affects viewing angles, occlusion handling and inspection distance |
| Detection Method | QL-03 (90% detection), EN-01 (lighting robustness) | Affects detection reliability under varying conditions |
| Cluster Identification | QL-05 (90% pick rate), RL-02 (software reliability) | Determines tracking accuracy across multiple scan positions |
| Manipulator Configuration | SR-01 (520mm reach), WT-03 (tip deflection <10mm) | Affects workspace coverage and approach trajectory flexibility |
| Gripper Design | SR-04 (50-60mm opening), QL-04 (cycle time <60s) | Determines grasp success rate and pick efficiency |
| Reservoir Design | SR-03 (15×15×15cm), ER-03 (tool-free removal <15s) | Affects storage capacity and operator interaction |

The following subsections present the alternative options for each decision point, evaluate them against relevant criteria and justify the selected solution.

## 2.2.2 Design Decision Analysis


### Decision Point 1: Camera Placement

Camera placement affects viewing angles, inspection distance and occlusion handling. Two options were evaluated: (A) **Eye-in-Hand** — camera mounted on the end-effector, enabling close-up inspection (~35cm viewing distance) and arbitrary viewpoint positioning; (B) **Upper-Arm Mounted** — camera on a proximal link providing platform stability but with less positioning flexibility.

**Table 2. Camera Placement Decision Matrix**

| Criterion | Weight | PDS Ref. | Eye-in-Hand | Upper-Arm |
|-----------|--------|----------|-------------|-----------|
| Close-up Inspection Capability | 0.20 | QL-03 | 5 | 2 |
| Multi-angle Viewing | 0.15 | SR-02 | 5 | 3 |
| Occlusion Handling | 0.36 | QL-03, EN-01 | 5 | 2 |
| Platform Stability | 0.20 | QL-02 | 3 | 5 |
| Calibration Simplicity | 0.09 | MA-03 | 3 | 4 |
| **Weighted Total** | **1.00** | | **4.42** | **2.93** |

**Selected: Eye-in-Hand** — Multi-viewpoint positioning and close-up inspection are critical for reliable detection and accurate depth measurement. The stability trade-off is acceptable given controlled motion during scanning.

---

### Decision Point 2: Detection Method

Cotton boll detection converts camera images into bounding boxes identifying target locations. Two options were evaluated: (A) **Classical Computer Vision** — color segmentation (HSV thresholding), edge detection and blob analysis; computationally efficient but sensitive to lighting variations; (B) **Deep Learning (YOLO)** — CNN-based detection trained on labeled cotton datasets providing robust detection under varying lighting and backgrounds [12][22].

**Table 3. Detection Method Decision Matrix**

| Criterion | Weight | PDS Ref. | Classical CV | YOLO |
|-----------|--------|----------|--------------|------|
| Robustness to Lighting Variation | 0.18 | EN-01 | 2 | 5 |
| Detection Accuracy | 0.23 | QL-03 | 2 | 5 |
| Handling Complex Backgrounds | 0.41 | QL-03, RL-01 | 2 | 5 |
| Computational Cost | 0.18 | EN-02 | 5 | 3 |
| **Weighted Total** | **1.00** | | **2.54** | **4.64** |

**Selected: YOLO11** — Deep learning is essential for achieving 90% detection accuracy under field-like conditions. The model trained on Cotton-boll-and-cluster-2 dataset achieves >0.7 confidence. GPU inference on Jetson Orin NX provides real-time performance.

---

### Decision Point 3: Cluster Identification Strategy

During panoramic scanning, the same cluster may be detected from multiple viewpoints. Two options were evaluated: (A) **Multi-Frame Tracking (ByteTrack)** — video-based trackers maintaining identity across frames using motion prediction [25][26]; designed for continuous video with smooth object movement; (B) **World-Space 3D Clustering** — each detection converted to 3D world coordinates using depth and TF transforms, then clustered based on spatial proximity independent of camera motion.

**Table 4. Cluster Identification Decision Matrix**

| Criterion | Weight | PDS Ref. | ByteTrack | World-Space 3D |
|-----------|--------|----------|-----------|----------------|
| Stability Across Discrete Scan Positions | 0.32 | QL-05 | 2 | 5 |
| ID Consistency | 0.25 | RL-02 | 3 | 5 |
| Independence from Frame Rate | 0.32 | QL-04 | 2 | 5 |
| Implementation Complexity | 0.11 | MF-03 | 4 | 3 |
| **Weighted Total** | **1.00** | | **2.47** | **4.78** |

**Selected: World-Space 3D Clustering** — Testing revealed ByteTrack ID instability when camera moved significantly between scan positions. World-space clustering provides robust identification regardless of camera trajectory, using complete-linkage algorithm that produces tight clusters without chain-linking artifacts.

---

### Decision Point 4: Manipulator Configuration

The manipulator must provide sufficient reach while enabling approach trajectories that avoid plant obstacles. Two options were evaluated: (A) **4-DOF Arm** — base rotation, shoulder, elbow, wrist; basic positioning but limited approach angle flexibility; (B) **6-DOF Arm (Braccio)** — full position and orientation control enabling gripper approach from arbitrary angles.

**Table 5. Manipulator Configuration Decision Matrix**

| Criterion | Weight | PDS Ref. | 4-DOF | 6-DOF |
|-----------|--------|----------|-------|-------|
| Approach Trajectory Flexibility | 0.29 | QL-05 | 2 | 5 |
| Workspace Coverage | 0.23 | SR-01, SR-02 | 3 | 4 |
| Obstacle Avoidance Capability | 0.35 | SF-03 | 2 | 5 |
| Mechanical Simplicity | 0.13 | MA-01 | 5 | 3 |
| **Weighted Total** | **1.00** | | **2.62** | **4.51** |

**Selected: 6-DOF Braccio Arm** — Additional DOF enable obstacle-avoiding trajectories while positioning the gripper optimally. The Braccio provides 520mm+ reach satisfying SR-01.

---

### Decision Point 5: Gripper Design

The gripper must reliably grasp cotton bolls without damage or loss during transfer. Two options were evaluated: (A) **Parallel Jaw Gripper** — two opposing fingers with controlled grip force, 50-60mm opening for cotton clusters; (B) **Vacuum/Suction** — negative pressure pickup; effective on flat surfaces but cotton's fibrous texture reduces seal effectiveness.

**Table 6. Gripper Design Decision Matrix**

| Criterion | Weight | PDS Ref. | Parallel Jaw | Vacuum |
|-----------|--------|----------|--------------|--------|
| Reliability on Fibrous Material | 0.31 | QL-05 | 4 | 2 |
| Simplicity & Cost | 0.28 | MC-01, MF-01 | 5 | 4 |
| Grip Security During Transfer | 0.31 | QL-05 | 4 | 2 |
| 3D Printability | 0.10 | MF-01 | 5 | 3 |
| **Weighted Total** | **1.00** | | **4.38** | **2.66** |

**Selected: Parallel Jaw Gripper** — Provides reliable grasping with controlled force. The 50-60mm opening (SR-04) is achievable with servo-driven, 3D-printable fingers compatible with the Braccio gripper base.

---

### Decision Point 6: Reservoir Design

The reservoir collects harvested cotton and must support tool-free emptying (ER-03). Two options were evaluated: (A) **Flip-Top Bin** — hinged lid enabling one-handed operation and quick access; standard commercial bins available; (B) **Lid-Stay Bin** — removable lid requiring two hands to operate and set aside during deposit.

**Table 7. Reservoir Design Decision Matrix**

| Criterion | Weight | PDS Ref. | Flip-Top | Lid-Stay |
|-----------|--------|----------|----------|----------|
| Tool-Free Operation (<15s) | 0.35 | ER-03 | 5 | 3 |
| One-Handed Access | 0.25 | QL-04 | 5 | 2 |
| Secure Contents During Motion | 0.20 | RL-01 | 4 | 3 |
| Cost & Availability | 0.20 | MC-03 | 5 | 4 |
| **Weighted Total** | **1.00** | | **4.80** | **2.95** |

**Selected: Flip-Top Bin** — Meets the 15-second removal requirement and allows deposit without fully opening. A commercial bin (15×15×15 cm, 209 TRY) minimizes custom fabrication while meeting all requirements.

---

## 2.2.3 Final System Configuration

**Table 8. Selected Components Summary**

| Decision Point | Selected Option | Weighted Score | Key Justification |
|----------------|-----------------|----------------|-------------------|
| Camera Placement | Eye-in-Hand (Wrist) | 4.42 | Enables multi-angle inspection and close-up viewing |
| Detection Method | YOLO11 | 4.64 | Robust detection under varying lighting conditions |
| Cluster Identification | World-Space 3D Clustering | 4.78 | Stable across discrete scan positions, complete-linkage prevents chain-linking |
| Manipulator | 6-DOF Braccio Arm | 4.51 | Flexible approach trajectories, sufficient reach |
| Gripper | Parallel Jaw | 4.38 | Reliable on fibrous material, 3D printable |
| Reservoir | Flip-Top Bin | 4.80 | Tool-free access, one-handed operation |

The selected components integrate into a coherent system architecture where ROS2 provides the communication backbone, YOLO11 detection feeds into the world-space clustering pipeline, MoveIt2 plans collision-free motions for the 6-DOF arm and the parallel gripper executes picks depositing cotton into the removable reservoir.

Additionally, a web-based operator dashboard is under development to address ergonomic requirements (ER-01, ER-02) identified in Section 2.1. The dashboard will provide intuitive START/PAUSE/EMERGENCY controls, color-coded status indicators and real-time ML confidence visualization, enabling flexible operator positioning during demonstrations. Section 2.3 provides detailed design specifications for each selected component and their integration.

---