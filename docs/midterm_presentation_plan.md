# ME492 Midterm Presentation Plan

## Slide 1: Recap + Updated System Architecture
**Goal:** Quick recall of solution, key tech, + NEW real robot deployment plan

**Content:**
- One slide, concise — "Here is our updated system architecture"
- Recall: Doosan M1013, Robotiq Hand-E, eye-in-hand RGB-D, YOLO11, ROS2 Humble, MoveIt2, Gazebo Ignition Fortress
- Key decisions from last term (ref appendix): parallel jaw gripper, world-space 3D clustering, deep learning detection
- NEW additions:
  - Real robot deployment path: ROS1→ROS2 ecosystem integration
  - Doosan driver stack, Docker containerization
  - Hardware interface layer

**Visual:** Updated System Architecture HTML diagram (full screen behind speaker)
- Find existing HTML in docs, update with real robot integration layer

---

## Slide 2: Full Pipeline Visualization + Demo Video
**Goal:** Show the complete harvest cycle working end-to-end

**Sub-parts:**
- **2a - Video:** Embedded Gazebo recording of full cycle
  - Good camera angle of scene
  - Scan with cluster annotations visible
  - Pre-grasp boll detection (annotated images)
  - Pick → retract → reservoir → release → return
  - Mention sim time / realtime factor briefly

- **2b - Log-parsed Dashboard/Monitoring:**
  - Parse harvest logs into timeline visualization
  - State transitions: IDLE → SCANNING → APPROACHING → HARVESTING → RETURNING
  - Per-cluster summary: predictions, annotated images, pick results
  - Telemetry: arm positions, gripper states, timing per phase
  - Build as monitoring-style view from log history

**Note:** If time-constrained, video + parsed log summary can be separate slides

---

## Slide 3: Technical Highlights / Critical Points
**Goal:** Show technical depth and engineering decisions

**3 key points:**

1. **Adversarial Test — White Ball**
   - Placed white ball in scene to test false positive behavior
   - YOLO correctly does NOT detect it as cotton boll
   - Visual: single annotated image showing ball ignored

2. **IK Multi-Seed vs MoveIt OMPL Planners**
   - Our approach: compute IK with HOME + CURRENT seed, pick lowest cost
   - Why: OMPL RRTstar 5s timeout unreliable for precise targets
   - Direct joint goal after IK = predictable, fast, no sampling randomness

3. **Approach Vector Computation**
   - Horizontal approach orientation: TCP Z-axis points from base toward target
   - Critical for eye-in-hand camera to face cluster during pre-grasp
   - Enables boll detection at close range before grasping

**Visuals:** One image/diagram per point

**Appendix mention:** "Previous term's detailed analysis — YOLO training/eval, decision matrices, spatial pipeline details — available in appendix"

---

## Appendix (referenced, not presented)
- Previous term's full presentation (ME429)
- YOLO training metrics, confusion matrix, detection demos
- Decision matrices (detection method, cluster ID, gripper design)
- Spatial detection pipeline 6-step flowchart
- Validation results (1cm accuracy)

---

## Preparation Tasks
1. [ ] Find & update System Architecture HTML diagram
2. [ ] Speed up sim (step_size tuning, gripper fix)
3. [ ] Record full pipeline video from Gazebo
4. [ ] Build log parser / dashboard visualization
5. [ ] Create slides
6. [ ] Write presentation script
7. [ ] Prepare for Q&A (sim speed, next steps, real robot timeline)

## Potential Q&A Topics
- Sim speed: 3-7% realtime on WSL2, no GPU passthrough for Gazebo
- Why Doosan M1013 instead of Braccio? (reach, payload, industrial grade)
- Real robot timeline and remaining integration work
- Visual servoing plans for pre-grasp refinement
- Boll deduplication (11 duplicate detections per cluster)
