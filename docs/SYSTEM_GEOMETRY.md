# Cotton Harvesting Simulator — Geometry Reference

Single-source-of-truth for sizes and positions across the Husky+M1013
robot, the cotton field environment, and the scan pipeline. Everything
in meters, angles in degrees unless noted. World frame: ROS REP-103
(X forward, Y left, Z up).

---

## 1) Husky Mobile Base

| Component | Dimension | Notes |
|---|---|---|
| Body length (X) | **0.99 m** | Body extent X∈[−0.495, +0.495] |
| Body width (Y) | **0.67 m** | Body extent Y∈[−0.335, +0.335] |
| Body height (Z) | **0.30 m** | Body Z∈[0.13, 0.43] (deck top = 0.43 m) |
| Body mass | 50 kg | |
| Wheel radius | 0.165 m | |
| Wheel width | 0.10 m | |
| Wheel base (X) | 0.51 m | Front-to-rear wheel centers |
| Wheel track (Y) | 0.555 m | Left-to-right wheel centers |
| Wheel mass | 2 kg each | 4 wheels at ±0.255 X × ±0.2775 Y |
| `husky_base_link` XY origin | (0, 0) | At wheel-contact-patch geometric center |
| Deck top (`deck_link`) | Z = 0.43 m | At top of body |

Drive: skid-steer DiffDrive plugin, `wheel_separation=0.555`,
`wheel_radius=0.165`, `max_lin=0.50 m/s`, `max_ang=0.60 rad/s`.

---

## 2) Arm Mount + M1013 Arm

```
deck_link (z=0.43)
  └─ arm_mount_joint  origin (0.20, 0, 0.025)
       └─ arm_mount_link (0.20×0.20×0.05 box)
            └─ arm_to_mount_joint origin (0, 0, 0.025) → base_0 at z = 0.48
```

**Arm mount** sits at deck front-center (X = +0.20 from Husky center)
so the arm overhangs the front 10 cm. Base_0 of the arm is at:
**world (Husky_X + 0.20, Husky_Y, 0.48)** when Husky yaw = 0.

**M1013 6-DOF arm** (Doosan, ported from `dsr_description`):

| Link | Joint offset xyz (m) | Approx contribution to reach |
|---|---|---|
| base_0 → link1 | (0, 0, 0.1525) | +0.15 Z |
| link1 → link2 | (0, 0.0345, 0) | small |
| link2 → link3 | (0.62, 0, 0) | +0.62 X (main reach) |
| link3 → link4 | (0, −0.559, 0) | +0.56 (rotated) |
| link4 → link5 | small | |
| link5 → link6 | small | |
| link6 → tool0 → hand_e | small | |
| hand_e → tcp | (0, 0, 0.14) | +0.14 along approach |

**Nominal max horizontal reach from base_0: ~0.85 m**
**Nominal max vertical reach: arm base z=0.48 + ~0.85 m ≈ 1.33 m**
(beyond that, IK fails for most orientations).

---

## 3) Reservoir (boll drop bin)

```
deck_link
  └─ reservoir_joint origin (−0.30, 0, 0.10)
       └─ reservoir_link
            • Floor visual at z=−0.095
            • 4 wall visuals (no collision — collision was removed
              so MoveIt can path freely through the rear deck volume)
```

| Property | Value |
|---|---|
| Dimensions (Lx × Ly × Lz) | **0.40 × 0.40 × 0.20** m |
| Wall thickness | 0.01 m |
| Position (deck frame) | (−0.30, 0, 0.10) — center 30 cm behind deck origin |
| Top opening Z (world) | 0.43 + 0.10 = **0.53 m** |
| Bottom Z (world) | 0.43 − 0.10 = **0.33 m** |
| Collision | **None** (visual-only; bolls drop via Gazebo set_pose teleport) |

`arm_commander.go_to_reservoir`: Stage 1 → HOME, Stage 2 → joint1=±π
(face local −X), Stage 3 → IK to TF of `reservoir_link` + hover 0.30 m.

---

## 4) Camera (wrist RGB-D)

Mounted on `link6` (last arm link), offset (−0.14, 0, −0.02) with
rpy=(0, −π/2, 0) so camera +X = arm approach (+Z of link6).

| Property | Value |
|---|---|
| Resolution | 640 × 480 |
| Horizontal FOV | 1.57 rad = **90°** |
| Image topic | `/camera/color/image_raw` |
| Depth topic | `/camera/depth/image_raw` |
| Update rate | 30 Hz |
| Optical frame | `camera_optical_frame` (ROS optical: +Z forward) |

---

## 5) Cotton Field Layout (`cotton_demo.world`)

### 5.1 Cluster anchor grid

Two rows of 6 clusters each (12 total). Anchor positions are in WORLD frame:

| Cluster ID | Row | Col | X (m) | Y (m) | Variant assigned |
|---|---|---|---|---|---|
| cluster_A_01 | 0 | 0 | 0.0 | 0.0 | B_mixed_green |
| cluster_B_01 | 0 | 1 | 3.0 | 0.0 | D_sparse_green |
| cluster_C_01 | 0 | 2 | 6.0 | 0.0 | C_mixed_brown |
| cluster_A_02 | 0 | 3 | 9.0 | 0.0 | F_dry_brown |
| cluster_B_02 | 0 | 4 | 12.0 | 0.0 | A_mature_white |
| cluster_C_02 | 0 | 5 | 15.0 | 0.0 | E_tall_mature |
| cluster_A_03 | 1 | 0 | 1.5 | 6.0 | B_mixed_green |
| cluster_B_03 | 1 | 1 | 4.5 | 6.0 | D_sparse_green |
| cluster_C_03 | 1 | 2 | 7.5 | 6.0 | C_mixed_brown |
| cluster_A_04 | 1 | 3 | 10.5 | 6.0 | F_dry_brown |
| cluster_B_04 | 1 | 4 | 13.5 | 6.0 | A_mature_white |
| cluster_C_04 | 1 | 5 | 16.5 | 6.0 | E_tall_mature |

Row 2 X is half-period offset from Row 1 (orchard staggering).

| Parameter | Value |
|---|---|
| In-row column spacing | **3.0 m** |
| Between-row spacing | **6.0 m** |
| Row 1 yaw | 0 (bolls face +Y, toward aisle) |
| Row 2 yaw | π (bolls face −Y, toward aisle) |

### 5.2 Husky aisle

| Param | Value |
|---|---|
| Aisle Y (scout_y) | **1.0 m** |
| Husky orientation in aisle | yaw = 0 (faces +X = drive direction) |
| Aisle clearance to Row 1 plants | scout_y − Husky_body_half = 1.0 − 0.335 = 0.665 m |
| Aisle clearance to Row 2 plants | row2_y − scout_y − body_half = 6 − 1 − 0.335 = 4.665 m |

Spawn pose: **(0, 1.0, 0)** with yaw = 0 (in front of cluster_A_01).

### 5.3 Plant variants (branch_variant_X)

Native model heights from bundle (in meters at scale 1):

| Variant | Native max boll Z | Native max boll Y | Pickable bolls |
|---|---|---|---|
| A_mature_white | 0.614 | +0.044 | 6 |
| B_mixed_green | 0.690 | +0.150 | 4 (+ baked-in green unripe visuals) |
| C_mixed_brown | 0.587 | −0.112 | 4 (+ baked-in brown dry visuals) |
| D_sparse_green | 0.504 | +0.054 | 3 (+ baked-in green unripe visuals) |
| E_tall_mature | 0.825 | −0.114 | 7 |
| F_dry_brown | 0.543 | −0.038 | 4 (+ baked-in brown dry visuals) |

**Scale normalization** (so every cluster's max boll Z = 1.51 m,
within arm reach budget):

| Variant | Effective scale | Max boll Z (world) |
|---|---|---|
| A_mature_white | 3.0 × 0.821 = **2.46** | 1.51 m |
| B_mixed_green | 3.0 × 0.730 = **2.19** | 1.51 m |
| C_mixed_brown | 3.0 × 0.859 = **2.58** | 1.51 m |
| D_sparse_green | 3.0 × 1.000 = **3.00** (reference) | 1.51 m |
| E_tall_mature | 3.0 × 0.611 = **1.83** | 1.51 m |
| F_dry_brown | 3.0 × 0.928 = **2.78** | 1.51 m |

Total pickable bolls in world: **56** (sum over per-cluster variants × 2 rows).

### 5.4 Cotton boll models (`cotton_pick_*`)

28 unique pickable boll meshes (`cotton_pick_A_01..A_06`,
`cotton_pick_B_01..B_04`, etc.). Each instance in the world is
named `<cluster_id>__<pick_id>` for unique Gazebo `set_pose` teleport.
**Collision: none** (visual mesh only — depth camera renders visuals,
not collision shapes; the cotton_pick mesh is a closed solid boll so
depth ray hits it properly).

### 5.5 Ground

| Layer | Geometry | Pose |
|---|---|---|
| `ground_plane` (physics) | Infinite plane, collision only | z=0 |
| `cotton_field_ground` (visual) | Mesh, **scale 4.0** | pose (0, 0, 0) |

Bundle ground mesh covers native X[0, 50] Y[0, 20]; at scale 4 the
visible patch covers X[0, 200] Y[0, 80].

---

## 6) Scan Sweep

Cluster scanner sweeps the arm in a pan × tilt grid from a SCOUT pose
(joint1=−π/2, joint5=+π/2 from HOME), capturing YOLO detections and
projecting them to world via depth.

| Axis | Joint | Angles (offset from SCOUT) | Count |
|---|---|---|---|
| Pan | joint1 | [−12°, 0°, +12°] | 3 |
| Tilt | joint5 | [**−32°**, −24°, −16°, −8°, 0°, +8°] | 6 |

(neg tilt = looks UP toward canopy top)

**Total: 3 × 6 = 18 poses per cluster scan.**

Per-pose settle time: 1.0 s. Each detection back-projected through
camera intrinsics (`fx=fy=277`, `cx=320`, `cy=240`) and transformed to
world via TF lookup `camera_optical_frame → world`.

Dedup radius after sweep: 0.05 m. Match radius (detection → YAML
inventory): 0.10 m.

---

## 7) Coordinate Sketch (top-down + side, ASCII)

### Top-down (X right, Y up, +Y is Husky's left)

```
       ROW 2 (Y=+6)
       ●—●—●—●—●—●        ← cluster_A_03..C_04 (X=1.5,4.5,...16.5)
       
       
       
       
       ↑ aisle ~6 m gap (mostly empty, Row 2 visual only by default route)
       
   ┏━━━┓                  ← Husky body 0.99 × 0.67 m
   ┃ H ┃   spawn (0, 1.0)
   ┗━━━┛                  yaw=0 (front +X)
       
       ROW 1 (Y=0)
       ●—●—●—●—●—●        ← cluster_A_01..C_02 (X=0,3,6,9,12,15)
       
   ↑ Y      → X
   spawn
```

### Side view at cluster_X (X plane, looking along −Y)

```
        Z
        ↑
  1.5 m ┤             ━━━ top boll
        │            ╲ │ ╱
  1.0 m ┤         branch (1.0–1.5m canopy)
        │            ╲│╱
  0.5 m ┤             │  ← stem
        │             │
  0.43m ┤━━━━━━ deck top ━━━━━━
        │ ┌──────────────┐     ┌──┐
   0.3m ┤ │   Husky body  │   ╱│  │
        │ │  0.99 × 0.67  │   │R │← reservoir
        │ │               │   ╲│  │
        │ └─┬──┬───┬──┬───┘     └──┘
  0.165m├──●  ●───●  ●           ← wheels
   0  ──┴──────────────────────────→ X (along row)
              wheel base 0.51 m
```

### Arm reach envelope (side view, Husky stopped at scout)

```
            top boll @ z=1.51 m   ← every variant after scale normalize
                  ●
                 ╱│ ← E variant reduced to 0.611× to fit
                ● │
               ╱  │
              ●   │
              │  /
   arm        ●  ← M1013 max horiz reach ~0.85m
   base       │
   z=0.48   ━━●━━ base_0
              ┃
   deck     ━━╋━━ z=0.43
              ┃
              Husky
```

---

## 8) Quick numerical recap

| Quantity | Value |
|---|---|
| Husky envelope | 0.99 × 0.67 × 0.30 m |
| Deck top Z | 0.43 m |
| Arm base Z | 0.48 m (deck + 0.05 mount) |
| Arm horiz reach | ~0.85 m |
| Max reachable Z | ~1.33 m (vertical) / ~1.51 m (with tilt) |
| All cluster top bolls | Z = 1.51 m (uniform after normalization) |
| Husky aisle Y | 1.0 m |
| Cluster spacing X | 3.0 m |
| Row spacing Y | 6.0 m |
| Ground patch | 200 × 80 m (centered loosely at origin) |
| Scan poses per cluster | 18 (3 pan × 6 tilt, tilt up to −32°) |
| Pickable bolls per world | 56 |
| Reservoir | 0.40 × 0.40 × 0.20 m, no collision, X=−0.30 deck rear |
