---

---

### Section 2.2: Overview of Possible Solutions (Decision Matrix)

- 2.2.1 Solution Space Overview
- 2.2.2 Alternative Solutions
    - A) Classical CV + Simple Manipulator
    - B) Deep Learning + Fixed Overhead Camera
    - C) Deep Learning + Eye-in-Hand RGB-D (CHOSEN)
- 2.2.3 Decision Matrix
- 2.2.4 Justification of Selected Approach

Decision Points to Cover (with sketches)

| Decision Point | Alternatives | Your Choice |
| --- | --- | --- |
| Vision System | Monocular / Stereo / RGB-D | RGB-D (depth + color) |
| Camera Placement | Fixed overhead / Eye-in-hand / Side-mounted | Eye-in-hand (wrist) |
| Detection Method | Color segmentation / Classical CV / YOLO | YOLO11 |
| Localization | 2D + depth lookup / Point cloud / Multi-view | 2D + depth back-projection |
| Arm DOF | 3-DOF / 4-DOF / 6-DOF | 4-DOF → 6-DOF (Braccio) |
| Scanning Strategy | Single viewpoint / Panoramic sweep / Continuous(our eventual goal→ real-time/fast process) | Panoramic 7×3 grid |
| Clustering | Pixel-space / World-space / Multi-frame cluster tracking via YOLO+Bytetrack /None | World-space complete-linkage |

Decision Matrix Criteria (Weighted)

| Criterion | Weight | Justification |
| --- | --- | --- |
| Detection Accuracy | 25% | Critical for autonomous harvesting |
| Localization Precision | 20% | Need ~1-2cm for grasping |
| Robustness to Occlusion | 15% | Cotton plants have complex geometry |
| Computational Cost | 10% | Real-time operation needed |
| Implementation Complexity | 15% | Senior project timeline |
| Scalability | 10% | Future field deployment |
| Cost | 5% | Academic budget |

Sketch Ideas for 2.2

1. Three camera placement configurations - simple top-view diagrams showing:
- Fixed overhead camera (limited angles)
- Side-mounted camera (occlusion issues)
- Wrist-mounted camera (follows arm, best coverage)
2. Detection approach comparison - showing same scene with:
- Color segmentation (fails on varying lighting)
- Classical CV (edge detection, unreliable)
- YOLO (robust bounding boxes)
3. Localization approaches - diagram showing:
- 2D only (no depth info)
- Stereo triangulation (needs baseline)
- RGB-D direct measurement (your approach)

---

---

### Section 2.3: Detailed Design and Analysis

Recommended Structure

2.3.1 System Architecture
- Package/Node diagram
- Data flow pipeline
- Communication topology (topics/services)

2.3.2 Mechanical Design
- Kinematic chain (DH parameters or transformation matrices)
- Joint limits table
- Workspace analysis / reachability
- End-effector frame definitions

2.3.3 Vision System
- Camera specifications table
- Pinhole camera model (equations)
- Intrinsics matrix K
- YOLO model specifications

2.3.4 Spatial Detection Pipeline
- Pixel-to-3D transformation (analytical)
- TF chain diagram
- Complete-linkage clustering algorithm
- Merge radius calculation

2.3.5 Scanning Strategy
- Panoramic scan grid (7×3 snake pattern)
- Joint-space coverage analysis
- FOV overlap calculation

2.3.6 Control & Motion
- Visual servoing control law

- Pipeline Integration: End-to-End Demonstration: 

visual servoing pipeline backing YoloSpatialDetectionPipeline with a relatively hard task: in a view the cluster will be visible partially in the lets

say bottm-right corner. we'll show images of 2 iter of camera focus at each panaromic scan position -> at the end of the scan, where we kind of merge the cluster posiitons we've calculated

into like 3(this can change, our algo is deciding based on proximity). -> and compare it to the ground truth simply

- Proportional gain derivation
- Motion planning integration

2.3.7 Validation & Results
- Ground truth comparison

- Case Study: Partial Visibility Recovery
- Accuracy metrics (~1-2cm)
- Error analysis

2.3.X Operator Interface Design
- User requirements (ergonomics from slide 6)
- UI components and layout
- Real-time telemetry display
- Control functionality
- State machine visualization

Visuals we Can Generate from the App:

- in app state transitions
- ML model confidence etc
- start stop controlling robot workflow
- emergency stop → home position

### 2.3.X Operator Interface

The RoboCot monitoring application provides real-time system
supervision and control capabilities, designed according to
ergonomic principles (Section 2.1.X).

### Design Requirements

- Clear system health indication at a glance
- Intuitive control buttons for START/PAUSE/EMERGENCY
- Real-time ML confidence visualization
- Pipeline progress tracking
- Alert logging for post-session analysis

### User Interface Components

[Figure X: RoboCot App Interface Layout]

The interface consists of five main sections:

1. **Status Banner**: Color-coded state indication
    - Green: Normal operation
    - Yellow: Evaluation/Warning
    - Orange: Compression/Maintenance
    - Red: Emergency stop
2. **Session Metrics**: Quantitative harvest progress
    - Bolls harvested count
    - Pick success rate (target: >90%)
    - Reservoir fill percentage
3. **Current Operation**: Detailed state information
    - Main state and substate text
    - ML confidence bar (color-coded: green >0.8, yellow >0.6, red <0.6)
4. **Pipeline Flow**: Visual progress through harvest cycle
    - Five sequential steps: Detect → View → Harvest → Transfer → Compress
    - Active step highlighted, completed steps marked
5. **Control Panel**: Operator commands
    - START/RESUME: Begin or continue operation
    - PAUSE: Safe stop with position retention
    - SKIP CLUSTER: Bypass current cluster
    - EMERGENCY STOP: Immediate halt

### State Machine

[Figure Y: Harvester State Machine Diagram]

The system implements a finite state machine with the following
primary states: IDLE, DETECTING_CLUSTERS, CLUSTER_VIEW_POSITION,
DETECTING_BOLLS, HARVESTING, TRANSFERRING, COMPRESSION, and
CLUSTER_COMPLETE.

---

---

### Visuals We Can Generate from Simulation

HIGH PRIORITY (Must-have)

| Visual | Source | Purpose |
| --- | --- | --- |
| Gazebo environment screenshot | Simulation | Show cotton field + robot setup |
| 7×3 Panoramic scan grid diagram | Draw from CHECKPOINT data | Show snake pattern coverage |
| YOLO detection annotated image | yolo_output/detect_*.png | Show bounding boxes |
| RGB + Depth side-by-side | Capture from Gazebo | Show camera outputs |
| Kinematic chain diagram | Draw from URDF | Show links/joints/frames |
| TF tree visualization | ros2 run tf2_tools view_frames | Show frame chain |
| Node interaction diagram | Already in CHECKPOINT | System architecture |
| Data flow pipeline | Already in CHECKPOINT | Detection pipeline |

MEDIUM PRIORITY (Nice-to-have)

| Visual | Source | Purpose |
| --- | --- | --- |
| Workspace reachability envelope | Calculate or RViz | Show arm reach |
| MoveIt path planning | RViz screenshot | Show planned trajectory |
| Cluster visualization (world-space) | Draw from validation results | Show 3D positions |
| Visual servoing convergence plot | If you have data | Show pixel error reduction |
| Camera FOV overlap diagram | Calculate from intrinsics | Justify scan grid |

---

---

📊 Tables Already Ready (from CHECKPOINT)(need to update for 6DOF braccio)

1. Joint limits table (Section 3.1)
2. Link dimensions table (Section 3.2)
3. Camera intrinsics table (Section 3.2)
4. Controller configuration table (Section 3.3)
5. Ground truth cluster positions (Section 7.2)
6. Inter-cluster distances (Section 7.2)
7. Validation results (Section 8.4)

---

---

✏️ Analytical Formulations to Include

1. Pinhole Camera Model

[u]   [fx  0  cx] [X/Z]
[v] = [0  fy  cy] [Y/Z]
[1]   [0   0   1] [ 1 ]

1. Back-Projection (Pixel → Camera Frame)

X_cam = (u - cx) × Z / fx
Y_cam = (v - cy) × Z / fy
Z_cam = depth

1. Visual Servoing Control Law

Δhip = -K_hip × (u - u_center)
Δshoulder = K_shoulder × (v - v_center)
Δelbow = -K_elbow × (v - v_center)

where K_hip = 0.002, K_shoulder = 0.0015, K_elbow = 0.001

1. Complete-Linkage Clustering Condition

boll ∈ cluster_i ⟺ ∀ member ∈ cluster_i: ||pos_boll - pos_member||_XY < r_merge

where r_merge = 0.25 × min(d_ij) = 0.25 × 0.485m = 0.121m

1. Forward Kinematics (DH or direct)

You can derive from your URDF - transformation matrices for each joint.

---

---

💡 Key Recommendations

1. For Decision Matrix

Don't just list alternatives - justify quantitatively:

- "RGB-D provides direct depth measurement, eliminating stereo calibration complexity"
- "Eye-in-hand camera enables close-up inspection at ~35cm viewing distance"
- "YOLO11 achieves 0.7+ confidence on cotton bolls vs 0.3 with color segmentation"
1. For Visuals

Always explain figures in text before they appear (rubric requirement):
"Figure 5 shows the 7×3 panoramic scan pattern. The snake traversal minimizes joint travel while ensuring complete FOV coverage..."

1. For Numerical Results

Highlight your ~1-2cm accuracy achievement:

- This is a strong quantitative result
- Show ground truth vs detected table
- Calculate and report error metrics (mean, std)
1. For Analytical Sections

Balance equations with intuition:

- Don't just dump formulas
- Explain why each formula matters
- Connect to practical implications

---

---

The "RoboCot App" is a real-time monitoring and control interface with:

| Component | Purpose |
| --- | --- |
| Status Banner | Current state (IDLE/DETECTING/HARVESTING/etc.) with color coding |
| Session Metrics | Bolls harvested, success rate %, reservoir fill level |
| Current Operation | Main state, substate, ML confidence bar |
| Pipeline Flow | Visual 5-step progress: Detect → View → Harvest → Transfer → Compress |
| Alerts Section | Rolling log of system events |
| Control Panel | START, PAUSE, SKIP CLUSTER, EMERGENCY STOP |

---

---

midterm sunumda yolo + bytetrack dedik fakat memory buffer sorunlarından dolayı iyi çalışmadı o yüzden kendi labelingimizi yapıyoruz 3D mapping ile.