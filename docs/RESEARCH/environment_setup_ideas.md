## Understanding the Layout

After reviewing your diagram and clarifications, the layout is straightforward:

```
        Y+ (Left)                                      Y- (Right)
           │                                              │
           │◄────────── EXPLORE PATH (Y+ to Y-) ────────►│
           │                                              │
           │         cluster_3    cluster_2    cluster_1  │
           │             ●            ●            ●      │
           │                                              │
           │                      X+ (Forward)            │
           │                         ▲                    │
           │                         │                    │
      reservoir                      │                    │
           □                      ROBOT                   │
           │                      (0,0)                   │
           │                                              │
```

**Key points:**
- Robot at origin, facing X+ (forward towards clusters)
- Clusters arranged in a line at X ≈ 0.75-0.85, spread across Y axis
- Explore path = camera sweeps from Y+ (left) to Y- (right)
- Reservoir at Y+ (to robot's left)

**Current `cotton_field.world` is already correct:**
- plant_1: (0.75, -0.45, z) - right side
- plant_2: (0.85, 0.0, z) - center
- plant_3: (0.75, +0.45, z) - left side
- reservoir: (0.0, +0.6, z) - left of robot

## Architecture Decision: Where to Define What

| Component | Location | Reasoning |
|-----------|----------|-----------|
| Visual geometry (plants, reservoir, markers) | `.world` file | Gazebo needs it for physics/rendering |
| Semantic positions (named targets) | `environment_config.yaml` | Code reads this, not hardcoded |
| TF frames for landmarks | Published by node from YAML | Enables tf2 lookups in perception pipeline |
| Explore waypoints | Generated from config | Not hardcoded, derived from start/end + num_points |

## Implementation Plan

### Phase 1: Environment Configuration (Foundation)

**1.1 Create `src/robot_arm/config/environment_config.yaml`**

Single source of truth for all semantic positions. Contains:
- Environment bounds (workspace limits)
- Fixed landmarks (reservoir, explore_start, explore_end)
- Cluster approximate positions (vision will refine)
- Exploration parameters

**Reasoning:**
- Separates "what Gazebo renders" from "what code knows"
- Easy to change positions without editing multiple files
- Can have different configs for different field layouts

**1.2 Update `cotton_field.world`**

Add visual markers for explore_start and explore_end (small spheres like existing markers).

**Reasoning:**
- Visual debugging - see where scan starts/ends in Gazebo
- Verify YAML config matches world visually

### Phase 2: Camera Setup

**2.1 Add `camera_link` + `camera_optical_frame` to URDF**

```xml
hand → camera_link → camera_optical_frame
```

**Reasoning:**
- Perception pipeline needs a frame to publish detections in
- `camera_optical_frame` follows ROS convention (Z forward, X right, Y down)
- Required for depth back-projection: pixel (u,v) + depth → 3D point

**2.2 Add Gazebo RGB-D camera plugin**

Publishes `/camera/color/image_raw` and `/camera/depth/image_raw`.

**Reasoning:**
- Can't do vision without camera topics
- Depth enables 3D localization of detected bolls

### Phase 3: TF Infrastructure

**3.1 Create `landmark_tf_publisher` node**

Reads `environment_config.yaml`, publishes static TF frames:
- `world → cluster_1`, `world → cluster_2`, `world → cluster_3`
- `world → reservoir`
- `world → explore_start`, `world → explore_end`

**Reasoning:**
- Any node can do `tf_buffer.lookup_transform("base_link", "cluster_1")`
- Decouples position knowledge from action logic
- Standard ROS pattern for known landmarks

### Phase 4: Update arm_commander

**4.1 Load targets from YAML instead of hardcoding**

Remove hardcoded `named_targets` dict, load from `environment_config.yaml`.

**Reasoning:**
- Single source of truth
- Change positions in one place, all nodes see it

**4.2 Add exploration waypoint generation**

Given `explore_start`, `explore_end`, `num_viewpoints` → generate intermediate poses.

**Reasoning:**
- Explore path is parameterized, not hardcoded
- Easy to adjust density of scan

### Phase 5: Explorer Node (Future)

Orchestrates the scan-detect-harvest loop:
1. Move to next scan waypoint
2. Capture image, run detection
3. If boll found → compute 3D position from depth → plan grasp
4. Repeat until explore_end

---

## File Structure After Implementation

```
robot_arm/
├── config/
│   └── environment_config.yaml    # NEW: Single source of truth
├── urdf/
│   └── mybot.urdf.xacro           # UPDATED: Add camera_link
├── worlds/
│   └── cotton_field.world         # UPDATED: Add explore markers
├── launch/
│   └── bot.launch.py              # UPDATED: Load config, spawn TF publisher
└── robot_arm/
    └── landmark_tf_publisher.py   # NEW: Publishes static TF frames
```

## Starting Implementation Now

Beginning with Phase 1: Environment Configuration.

---

## Corrections Applied (User Feedback)

### 1. Single Source of Truth - No Duplication
**Problem:** If YAML and .world both define positions, they can drift apart.
**Solution:** YAML is the single source of truth. Update .world markers manually to match (acceptable for toy project).

### 2. Use `base_link` for Targets, Not `world`
**Problem:** MoveIt uses `base_link` as planning frame. Using `world` adds unnecessary transforms.
**Solution:**
- Perception outputs in `camera_optical_frame`
- Transform targets into `base_link` for planning
- `world` only for Gazebo visuals and global anchor

Since robot spawns at (0, 0, 0.1), the offset is minimal but cleaner to stay in `base_link`.

### 3. MoveIt Collision Objects - Critical!
**Problem:** MoveIt doesn't see Gazebo objects. Arm will plan paths through reservoir/plants.
**Solution:** In Phase 3, `landmark_tf_publisher` also publishes `CollisionObject` messages to `/planning_scene`:
- Reservoir as a box
- Plant stems as cylinders (optional)

### 4. Camera Placement
**Solution:** Attach `camera_link` to `tool0` (not `hand`) with offset to avoid gripper clipping.

---

## Updated Plan Summary

```
YAML (environment_config.yaml)
     │
     ├──► landmark_tf_publisher ──► Static TF frames (in base_link)
     │                          └──► CollisionObjects to /planning_scene
     │
     └──► arm_commander ──► Loads targets, plans in base_link

.world file ──► Gazebo visuals only (manually synced with YAML)
```
