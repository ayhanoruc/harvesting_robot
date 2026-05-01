# Phase 2 — Codebase Audit & Migration Map

**Date:** 2026-04-17
**Source:** Full pass over URDF, launch, MoveIt, controllers, all orchestrator nodes, world file, configs.
**Goal:** Inventory what we keep, what we extend, what we rewrite, what we delete — before touching any code.

---

## A. Mevcut Mimarinin Net Resmi

### A.1 World (`robot_arm/worlds/cotton_field.world`)
- **Boyut:** 10×10m ground plane, hardcoded 3 plant + 3 boll + 1 reservoir.
- **Plant'lar:** Static cylinder stem + DAE cluster mesh (model://cotton_cluster). Sabit pozisyonlar (0.875,±0.475) ve (0.975,0).
- **Bolls:** 3 dinamik sphere (r=0.035m, mass=0.01kg, mu=1.0). Cluster center'larina pose ile yerlestirilmis.
- **Reservoir:** Static box 0.3×0.3×0.2 @ (0,0.6,0.1).
- **Physics:** DART, step 0.005, real_time_factor 1.0.
- **Phase 2'de:** Tamamen **silinecek**. Yerine yeni `orchard.world` (orchard mesh + Husky spawn) gelecek. Hardcoded boll/plant gitmeli, dinamik spawner ile yenilensiz.

### A.2 URDF (`robot_arm/urdf/m1013_robocot.urdf.xacro`)
- **Root:** `world` (fixed) -> `base_0` @ (0,0,0.1). M1013 base 10cm yerden yukseklikte.
- **Chain:** world -> base_0 -> link1..6 -> tool0 -> hand_e_link -> hande_left/right_finger -> tcp(@0.14m).
- **Camera:** link6'ya monteli, `xyz="-0.14 0 -0.02" rpy="0 -1.5708 0"`. RGB-D Ignition sensor, 30Hz, 640×480, 90° HFOV, K matrix hardcoded (fx=fy=277, cx=320, cy=240). Clip 0.05-3m. **Range cok kisa orchard icin** — agac satirina bakarken 3m yetmez, en az 5-7m olmali.
- **Gripper:** Left finger driven, right finger mimic. ros2_control sadece left'i sees. Friction high (mu=1.0).
- **ros2_control:** Tum joint'ler position interface. `ign_ros2_control/IgnitionSystem` hardware plugin.
- **Phase 2'de:**
  - **Korunur:** M1013 chain, hand-e mimic, camera optik frame yapisi.
  - **Degisir:** `world -> base_0` joint kalkacak. Yerine `world -> husky_base_link -> ... -> mount_plate -> base_0` zinciri gelecek. Camera clip far'i 7m'ye cikar. Reservoir URDF'e link/joint olarak eklenecek (Husky deck'inde fixed).
  - **Eklenecek:** Husky URDF (clearpath ros2'den), mount plate link (0.10m yukseklik), reservoir box link.

### A.3 Controllers (`robot_arm/yaml/controllers.yaml`)
- 100Hz update, joint_state_broadcaster + arm_controller (joint1-6) + gripper_controller (left finger).
- **Phase 2'de:** Husky'nin diff_drive_controller eklenecek. Joint listesi husky_left_wheel + husky_right_wheel (velocity interface, position degil).

### A.4 MoveIt (`robot_arm_moveit_config/`)
- **SRDF planning groups:** `arm` (base_0 -> tool0), `gripper` (left finger).
- **Named poses:** home (j2=-0.922, j3=2.4494, j5=-1.3 — kameranin asagi-on baktigi pozisyon), ready, zero, open, closed.
- **Disable_collisions:** Adjacent links + camera + tcp marker. Husky/mount eklenince yeni collision pair'ler lazim.
- **Phase 2'de:** Planning frame `world` -> `husky_base_link` mi olmali? **Hayir** — `world` kalsin, `base_0` Husky uzerine fixed olarak monte edilince TF zinciri kendiliginden cikar. SRDF chain `base_0 -> tool0` aynen kalir.
- **Yeni named pose'lar gerekli:**
  - `scout_left` — kol satir solunda dik bakacak (j1 ~+90°, j5 belirlenecek)
  - `scout_right` — sag-yan bakis
  - `harvest_ready` — pre-grasp icin neutral

### A.5 Launch (`robot_arm/launch/bot.launch.py`)
- Gazebo + RSP + spawn + ros_gz_bridge + ros_gz_image + 3 controller (sequenced) + landmark_publisher.
- **Phase 2'de:** Husky ekleme `spawn_entity` argumanlarina yansiyacak (-z 0.5). Bridge yeni topic'ler ekleyecek (Husky odom, cmd_vel, lidar varsa).

### A.6 Orchestrator (`orchestrator/orchestrator/`)

| Node | Lines | Sorumluluk | Phase 2 Durumu |
|------|-------|-----------|---------------|
| `main.py` | 819 | Top-level FSM: IDLE -> SCANNING -> per-cluster (APPROACHING -> HARVESTING) -> RETURNING | **Genisletilecek**: NAVIGATING + DETECTING_ON_MOVE state'leri eklenecek, route waypoint loop. |
| `harvest_executor.py` | 408 | 8-step pick-place. **Gripper bypassed (line 351-353): hep success doner.** | **Korunur** — sadece gripper bypass kaldirilip gercek call'lar acilacak, lift/reservoir koordinati Husky deck'ine gore guncellenecek. |
| `arm_commander.py` | 615 | IK multi-seed (HOME + current), MoveGroup OR direct trajectory, normalize+validate. | **Korunur** — sadece HOME_JOINTS world frame'e tied (sabit base bos boslukta), Husky uzerinde aynen calisacak (TF chain icinden gecer). Validation `j1 > 135°` cok kisitli (scout pose'lar icin gevsetilmeli). |
| `explorer.py` | 832 | Panoramic scan: 7×3 grid OR 3 j1 pozisyonu (harvest_pipeline.launch'ta 3-pos). Direct JointTrajectory publish (MoveIt yok). | **Yeniden tanim** — orchard'da satir-bazli scan farkli olur. "Cluster scan from scout pose" mantigi farkli. **Buyuk parca refactor edilecek** veya yeni `cluster_scout` node yazilacak (eski explorer arsiv). |
| `real_yolo_detector.py` | 426 | YOLO inference, /yolo/detect (raw) + /yolo/detect_clusters (pixel-merge). Annotated PNG kaydeder. | **Korunur** — model retrain edilince path'i swap. Cluster pixel distance (150) orchard'a uyarlanir. |
| `spatial_detection_pipeline.py` | 896 | YOLO+focus+depth coordinator. World-space complete-linkage clustering, ground truth validation, image saving. | **Korunur (logic)** — ground_truth source farkli olacak (orchard tree positions YAML, 219 entry). Merge_radius_factor cluster spacing'e gore yeniden auto-calc edilecek. |
| `depth_processor.py` | 282 | Pixel -> 3D, K matrix bypass (P matrix bug fix). | **Korunur (degisiksiz)** — kamera intrinsic'i URDF'den geldigi icin auto-adapt eder. |
| `camera_focus.py` | 249 | Pixel error -> j1/j2/j3 proportional adjustment. | **Korunur, simdilik kullanilmiyor** (focus_iters=0). Mobile base ile yeni "view_adjust" ihtiyaci varsa burayi kullaniriz. |
| `gripper_controller.py` | 164 | JointTrajectory publish + pos polling. **Suanda calisiyor ama harvest_executor'da bypass edilmis.** | **Korunur** — bypass kaldirilacak, real grip Phase 2 hedefi. |
| `mock_yolo_detector.py` | ? | Sentetik detection (gercek yolu olmadiginda). | **Silinebilir** — eski. |

### A.7 Custom interfaces (`harvester_interfaces/`)
- `BoundingBox` (u/v_min/max, conf, label, area)
- `DetectedCluster` (id, position, conf, area, num_detections, scan_position)
- `YoloDetect`, `PixelTo3D`, `FocusFromPixel`, `FocusFromPosition`, `RunDetectionPipeline`, `HarvestBoll`, `GetDetectedClusters`
- **Phase 2'de:** `RunDetectionPipeline` ve `FocusFromPosition` kullanilmiyor — silinebilir. Yeni interface'ler ihtiyaci olabilir: `NavigateToCluster` (id, view_pose), `BollState` (id, ripe, picked) gibi.

### A.8 Environment Config (`robot_arm/config/environment_config.yaml`)
- Hardcoded 3 cluster, reservoir, explore_start/end. Workspace bounds. Collision objects empty.
- **Phase 2'de:** Tamamen yeniden yazilacak. Yapi:
  - `orchard.tree_positions` (219 entry, id+x+y+z_canopy_min+z_canopy_max)
  - `husky.deck_height`, `arm_mount.offset`, `reservoir.local_pose`
  - `nav.route_waypoints` (yaml liste)
  - `vision.scout_pose_left/right` (joint configs)

---

## B. Boll Spawn Analizi (Yeni Konu)

Mevcut world hardcoded 3 boll. Phase 2'de:
- **Spawner script** lazim — `boll_spawner.py` (yeni). 219 agac × 5-7 boll = ~1000-1500 obj.
- Performans riski: 1500 dynamic obj Gazebo'yu sleyebilir. Cozum:
  - Robot konumuna gore "active radius" — sadece 3-5m yarıcaptaki bollar dinamik, gerisini static (model attribute).
  - Spawn anında belirleyip toggle etmek mumkun (entity remove + re-add) ama maliyetli. **Daha iyi:** Hepsi static dur, robot yakininkiler `<static>false</static>` ile spawnla.
- Boll varyasyon:
  - Ripe (beyaz): r=0.035m, RGB ~(0.95, 0.95, 0.9)
  - Unripe (yesil-kahve): r=0.025m, RGB ~(0.4, 0.6, 0.3) veya (0.6, 0.4, 0.2)
  - Random %70/30 dagilim
- Z konumlandirma: tree position'a gore 1.5-2.5m arasi (canopy lower-mid).
- Branch noise: trunk x,y +- 0.3-0.5m random offset (canopy genisligi).

---

## C. Husky Stack Aciklamasi

Husky ROS2 destegi:
- `clearpath_simulator` (ROS2 Humble, Gazebo Garden/Ignition support)
- Veya manuel: `husky_description` + URDF'den base_link + 4 wheel + plate ekleyip kendi hardware/diff_drive controller'imizi yazariz
- Nav2 stack hazir, Husky template'i mevcut (`nav2_bringup`)

**Onerilen yol:** Manuel kompozisyon, cunku:
1. clearpath_simulator buyuk dependency, gerek yok
2. Husky URDF'i tek kullanilik fork edip mount plate eklemek temiz
3. Diff drive controller ros2_control uzerinden zaten yapilabilir

---

## D. Mevcut Eksiklikler / Tehlike Sinyalleri

| # | Sorun | Etki | Phase 2 Eylemi |
|---|-------|------|----------------|
| D1 | `arm_commander.validate_joints` j1 > 135° rejecte ediyor (line 311) | Scout pose'larda j1 ~90° ihtiyaci olabilir, sinir limit | Validation gevset, scout pose'larda bypass param ekle |
| D2 | `harvest_executor._call_gripper` bypass'li (line 351-353) | Real grip yok, sadece hareket gosteriyor | Bypass kaldir, gripper_controller'a gercek call ac |
| D3 | Camera clip far=3m | Orchard satirina bakarken 3m yetmez | URDF'te 7-10m'ye cikar |
| D4 | Reservoir URDF'te link degil, world'de static box | Husky uzerine binince hareket etmeli | URDF'e fixed joint ile ekle (deck child'i) |
| D5 | `pre_grasp_offset` hem orchestrator hem arm_commander'da set ediliyor (eski bug izi) | Tasks_today.md'de "double offset" bug not'u var | Orchestrator'da hesapla, arm_commander'a hazir geometri ver — arm_commander offset uygulamasin |
| D6 | World-frame TF "world" sabit, base_0 (0,0,0.1)'de fixed | Husky hareket edince base_0 da hareket etmeli | URDF'te world->husky_base_link odom-driven, husky->base_0 fixed |
| D7 | `arm_commander.HOME_JOINTS` world-frame agnostic ama spawn pozisyonuna bagli | Husky uzerinde HOME tanimi degisecek mi? Hayir, joint-space sabit | Test et — IK target pozisyonlari Husky base'e relative olmali, world degil |
| D8 | `_phase_scanning` (orchestrator/main.py) "for demo: always uses config positions" comment'i (line 406-408) | Vision sonuclari kullanilmiyor, demo icin shortcut | Orchard'da config positions = 219 tree, vision filtresi gercekten kullanilacak |
| D9 | `mock_yolo_detector.py` halen package'da | Iki YOLO node ayni topic kullanirsa carpisma | Sil veya disable. |
| D10 | `cotton_cluster` model `model://cotton_cluster/cluster.dae` reference — bizim repo'da yok mu? | Plant cluster mesh import dosyasi nereye | Eger gerekmiyorsa world'den cikar; gerekiyorsa model dizinine ekle |

---

## E. Phase 2 — Konkret Migration Map

### E.1 Silinecek Kod / Asset
- `robot_arm/worlds/cotton_field.world` (yeni `orchard.world` ile replace)
- `orchestrator/orchestrator/mock_yolo_detector.py`
- `harvester_interfaces/srv/RunDetectionPipeline.srv`
- `harvester_interfaces/srv/FocusFromPosition.srv`
- `robot_arm/launch/bot.launch_old.py` (zaten _old suffix)
- `src/_legacy/` dizini (audit gerek)

### E.2 Korunacak Kod (TOUCHED)
- `arm_commander.py` (D1 fix, D5 fix)
- `harvest_executor.py` (D2 fix gripper bypass kaldir)
- `m1013_robocot.urdf.xacro` (Husky chain icine gomulecek, camera clip D3)
- MoveIt SRDF (yeni named pose'lar: scout_left/right/harvest_ready)
- `environment_config.yaml` (yeniden yazilacak orchard schema'siyla)

### E.3 Yeni Yazilacak
- `robot_arm/urdf/husky_robocot.urdf.xacro` — Husky + mount + M1013 + reservoir tek dosya. Mevcut `m1013_robocot.urdf.xacro`'yu xacro:include ile cagirir.
- `robot_arm/worlds/orchard.world` — orchard mesh URDF + lighting, dinamik bos.
- `robot_arm/scripts/boll_spawner.py` — orchard tree pozisyonlarini okuyup runtime'da boll spawn eder.
- `robot_arm/config/orchard_tree_positions.yaml` — 219 tree pozisyonu (extract_trees_v2.py output'undan).
- `robot_arm/config/orchard_environment.yaml` — Husky deck pose, mount pose, reservoir local pose, scout poses, route waypoints.
- `orchestrator/orchestrator/cluster_scout.py` (yeni) — wrist cam scan-on-the-move + cluster detection + stop decision. Eski `explorer.py` ya silinir ya arsiv.
- `orchestrator/orchestrator/nav_coordinator.py` (yeni) — Nav2 send_goal wrapper, route waypoint loop.
- Orchestrator FSM uzantisi: NAVIGATING ve DETECTING_ON_MOVE state'leri main.py icinde.

### E.4 Kontrol Noktasi (Sanity)
F1 jumpstart icin minimum:
1. Mesh portu (3 .dae bizim meshes/orchard/'a)
2. orchard_tree_positions.yaml uretildi
3. Minimal `orchard.world` (mesh visual + lighting + ground physics)
4. Mevcut `m1013_robocot.urdf` orchard.world'de spawn olabiliyor mu (Husky henuz yok, sabit base — sanity check)

Bu 1.5 isgununde tamam.

---

## F. Acik Sorular (Code-Level)

1. **`arm_commander.send_joint_goal_direct`** (line 573) — direct trajectory yontemi var ama default `use_direct_trajectory=False`. Phase 2'de Husky/scout pose'lari icin direct daha hizli mi (MoveIt OMPL bypass)?
2. **`explorer.py`'deki home_joints reference** (line 769) `[0.0000, -0.922, 2.4494, 0.0, 0.2708, 0.0]` — bu rotated j5 (kameranin clusterlara baktigi). Orchard'da scout pose buna karsilik gelir mi? Test gerekli.
3. **Cotton cluster mesh** (`model://cotton_cluster/cluster.dae`) — repo'da var mi? Yoksa world'den cikarmak gerek.
4. **Demo shortcut** (`_phase_scanning` line 406-408): vision sonuclari hep config'le ezildi — Phase 2'de bu shortcut kalkacak, gercek vision-driven flow olacak.
5. **`harvest_executor` step 6 reservoir hover** (line 207, `rz_hover = rz + 0.15`): Husky deck'i 0.39m, reservoir local 0.1m yukseklikte olursa world'de 0.49m, hover 0.64m — M1013 bu yuksege bakabilir mi? Reach test.

---

## G. Onerilen Klasor Yapisi (Phase 2 sonrasi)

```
src/
├── robot_arm/
│   ├── urdf/
│   │   ├── m1013_robocot.urdf.xacro       # arm + gripper + camera (sabit kalir)
│   │   └── husky_robocot.urdf.xacro       # YENI: husky + mount + arm + reservoir
│   ├── worlds/
│   │   ├── cotton_field.world             # SILINECEK
│   │   └── orchard.world                  # YENI
│   ├── meshes/
│   │   ├── m1013_blue/                    # KORUNUR
│   │   ├── m1013_collision/               # KORUNUR
│   │   ├── hande/                         # KORUNUR
│   │   └── orchard/                       # YENI: 3 .dae mesh
│   ├── config/
│   │   ├── controllers.yaml               # Husky controllers eklenecek
│   │   ├── orchard_environment.yaml       # YENI (eski environment_config replace)
│   │   ├── orchard_tree_positions.yaml    # YENI
│   │   └── gz_bridge.yaml                 # Husky topic'leri eklenecek
│   ├── launch/
│   │   ├── orchard.launch.py              # YENI (bot.launch.py replace)
│   │   └── bot.launch.py                  # eski olarak silinecek/_legacy
│   ├── scripts/
│   │   └── boll_spawner.py                # YENI
│   └── robot_arm/
│       └── landmark_publisher.py           # KORUNUR (orchard config'i okur)
├── robot_arm_moveit_config/
│   ├── config/
│   │   ├── robot_arm.srdf                  # scout_left/right/harvest_ready ekle
│   │   └── ... (kalanı korunur)
│   └── ... (kalanı korunur)
├── orchestrator/
│   └── orchestrator/
│       ├── main.py                         # FSM uzatilacak
│       ├── harvest_executor.py             # gripper bypass kaldir
│       ├── arm_commander.py                # D1 fix
│       ├── nav_coordinator.py              # YENI
│       ├── cluster_scout.py                # YENI (eski explorer.py replace)
│       ├── real_yolo_detector.py           # KORUNUR (model retrain ile path swap)
│       ├── spatial_detection_pipeline.py   # KORUNUR (config schema ile auto-adapt)
│       ├── depth_processor.py              # KORUNUR
│       ├── camera_focus.py                 # KORUNUR (kullanilmiyor, hazir)
│       ├── gripper_controller.py           # KORUNUR
│       ├── mock_yolo_detector.py           # SILINECEK
│       └── explorer.py                     # _legacy/'a tasinacak
├── harvester_interfaces/
│   ├── msg/                                # KORUNUR
│   ├── srv/
│   │   ├── ... (mevcut korunur)
│   │   ├── NavigateToCluster.srv           # YENI (belki)
│   │   └── BollState.srv                   # YENI (belki)
└── docs/
    ├── PHASE2_PLAN.md                      # mevcut
    ├── PHASE2_REPO_AUDIT.md                # bu doc
    └── ... (kalan docs korunur)
```
