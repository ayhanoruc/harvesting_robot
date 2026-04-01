# Midterm Sunumuna Kadar TODO

## Code Review & Bug Fix (2026-04-01)

TASK 1-6 implemente edildi, ardindan full code review yapildi. 4 bug bulundu ve fixlendi:

1. **CRITICAL: Cluster ID mismatch** — Vision scan "detected_cluster_0" uretir, arm_commander "cluster_1" bekler → arm hic hareket etmez. FIX: `_go_to_named` yerine `_go_to_xyz(pre_grasp, approach_orientation=True)` kullanildi.
2. **CRITICAL: Executor deadlock** — `rclpy.spin_until_future_complete` MultiThreadedExecutor'den node'u calar, orphan birakir → sonraki tum service call'lar deadlock. FIX: poll-wait `_wait_future()` metodu ile degistirildi (main.py 8, harvest_executor.py 5 yer).
3. **MODERATE: Double pre-grasp offset** — Orchestrator offset hesaplar, arm_commander tekrar offset uygular → 30cm geri gider. FIX: #1 ile birlikte cozuldu.
4. **MINOR: Kamera yonelimi** — `go_to_pose` orientation gondermiyordu → kamera clustera bakmiyor olabilir. FIX: arm_commander'a `use_approach_orientation` parametresi eklendi, orchestrator cluster approach'larda True gonderiyor.

Degisiklik yapilan dosyalar:
- `src/orchestrator/orchestrator/main.py` — _wait_future, _go_to_xyz approach_orientation, _phase_approaching fix
- `src/orchestrator/orchestrator/harvest_executor.py` — _wait_future
- `src/robot_arm_moveit_config/robot_arm_moveit_config/arm_commander.py` — use_approach_orientation param + go_to_pose'da kullanimi

---

## Full Data Flow: HOME → Harvest → HOME

### Trigger
```
ros2 service call /orchestrator/start_harvest std_srvs/srv/Trigger "{}"
```

### Phase 1: SCANNING (orchestrator → explorer → spatial_detection → YOLO + depth)
```
orchestrator._start_harvest_cb()
  → _set_state(SCANNING)
  → _phase_scanning()
      1. _go_home() → arm_commander /go_to_named "home" → joint goal HOME_JOINTS
      2. /detection/clear → spatial_detection_pipeline clears tracked_clusters
      3. /explorer/panoramic_scan → explorer spawns thread, returns immediately
         Thread runs:
           a. /detection/clear (tekrar, zararli degil)
           b. /detection/wait_ready (YOLO + depth hazir mi kontrol)
           c. 3 pozisyon icin (j1 = -0.50, 0.0, +0.50 rad):
              - JointTrajectory publish → /arm_controller/joint_trajectory
              - Wait move_duration + 0.5s (jointlar oturur)
              - Position name publish → /detection/current_position
              - Wait pan_pause_duration (2.0s kamera icin)
              - /detection/run_at_position → spatial_detection_pipeline:
                  → /yolo/detect_clusters → real_yolo_detector:
                      - YOLO inference (latest camera frame)
                      - Yakin boll bbox'lari grupla (pixel-space complete-linkage)
                      - Return: merged cluster BoundingBox[]
                  → Her cluster bbox icin:
                      - Pixel center (u, v) hesapla
                      - /depth_processor/pixel_to_3d → depth_processor:
                          - Back-project (u,v) with depth + K matrix
                          - TF: camera_optical_frame → world
                          - Return: 3D Point (world frame)
                      - Z offset correction uygula
                      - tracked_clusters'a ekle (world-space X,Y complete-linkage merge)
           d. HOME joints'e don
           e. /explorer/scan_status "COMPLETE" publish
      4. Orchestrator _scan_complete flag'i bekler (subscription: /explorer/scan_status)
      5. /detection/get_results → TrackedCluster[] (her cluster icin best detection position)
      6. Return: {"detected_cluster_0": [x,y,z], "detected_cluster_1": [x,y,z], ...}
```

### Phase 2: Per-cluster loop

#### 2a. APPROACHING (orchestrator → arm_commander → YOLO + depth)
```
_set_state(APPROACHING)
_phase_approaching(plan):
  1. Pre-grasp hesapla: cluster center'dan 15cm geri (approach vector boyunca)
     ornek: cluster at (0.875, 0.475, 0.50) → pre_grasp at (~0.743, ~0.403, 0.50)
  2. _go_to_xyz(pre_x, pre_y, pre_z, approach_orientation=True)
     → arm_commander params: target_x/y/z + use_approach_orientation=True
     → /go_to_pose → arm_commander:
         - compute_approach_quaternion() → tcp Z-axis hedefi gosterir (yatay)
         - compute_ik_multi_seed() → HOME seed + current seed, en az maliyet
         - validate_joints() → joint1 arkaya gitmesin
         - send_joint_goal() → MoveGroup action → arm hareket eder
         - _check_tcp_error() → pozisyon hatasi < 5cm mi kontrol
  3. camera_settle_time bekle (3.0s) — kamera stabilize olsun
  4. _detect_bolls():
     → /yolo/detect (RAW, clusters degil!) → bireysel boll bounding box'lar
     → Her detection icin ("cotton" veya "boll" iceren label):
         - Bbox'tan pixel center (u, v) hesapla
         - /depth_processor/pixel_to_3d → 3D world position
         - BollTarget(position=[x,y,z]) olustur
     → Return: BollTarget listesi
  5. Boll detect basarisiz → fallback: cluster center tek boll olarak kullan
```

#### 2b. HARVESTING (orchestrator → harvest_executor → arm_commander + gripper)
```
_set_state(HARVESTING)
_phase_harvesting(plan):
  Her boll icin:
    _pick_single_boll(boll, pre_grasp_view):
      → /harvest/pick_boll → harvest_executor._pick_boll_cb():
        8 adimli sekans:
          [1/8] PRE-GRASP:  _go_to_xyz(pre_grasp) → cluster view'e git
          [2/8] OPEN:       /gripper/open → gripper_controller → JointTraj → fingerlar acar
          [3/8] APPROACH:   _go_to_xyz(boll) → boll merkezine git
          [4/8] CLOSE:      /gripper/close → gripper_controller → fingerlar kapar
          [5/8] LIFT:       _go_to_xyz(boll.x, boll.y, boll.z + 0.15m) → yukari kaldir
          [6/8] RESERVOIR:  _go_to_xyz(0.0, 0.6, 0.2) → birakma noktasina git
          [7/8] RELEASE:    /gripper/open → bollu birak
          [8/8] RETURN:     _go_to_xyz(pre_grasp) → cluster view'e don

        Her adim: arm_commander'a param set → /go_to_pose veya /gripper/*
        Her adim: zamanlanir ve loglanir ("[3/8] APPROACH BOLL: reached in 12.3s")
        Omur istatistikleri: "PICK #2 SUCCESS: 45.3s total, 2/3 lifetime"
```

### Phase 3: RETURNING
```
_set_state(RETURNING)
_go_home() → arm_commander /go_to_named "home" → HOME_JOINTS

Ozet log:
  "HARVEST COMPLETE: 5/6 bolls, 312s wall time"
_set_state(IDLE)
```

### Node Topolojisi
```
                                          ┌─────────────────┐
                          /go_to_pose     │                 │
              ┌──────────────────────────►│  arm_commander  │◄── /compute_ik (MoveIt)
              │  /go_to_named             │  (IK+JointGoal) │──► MoveGroup action
              │  /set_parameters          │                 │
              │                           └─────────────────┘
              │
┌─────────────┴───────┐    /harvest/pick_boll    ┌──────────────────┐
│                     │◄────────────────────────►│ harvest_executor │
│   orchestrator_node │                          │  (8-step pick)   │
│   (state machine)   │                          └────────┬─────────┘
│                     │                                   │
└──┬──────┬───────┬───┘                           /gripper/open,close
   │      │       │                                       │
   │      │       │    /yolo/detect                       ▼
   │      │       └────────────────►┌───────────┐  ┌──────────────────┐
   │      │                         │ real_yolo  │  │gripper_controller│
   │      │  /detection/*           │ _detector  │  │ (topic publish)  │
   │      └────────────►┌───────────┴──┐         │  └──────────────────┘
   │                    │  spatial_    ││         │
   │                    │  detection   ├┘         │
   │                    │  _pipeline   │◄─────────┘
   │                    └──────┬───────┘  /yolo/detect_clusters
   │                           │
   │  /explorer/panoramic_scan │  /depth_processor/pixel_to_3d
   │                           │
   │  ┌────────────┐    ┌──────┴───────┐
   └─►│  explorer   │    │depth_processor│
      │ (3 pos scan)│    │(pixel→3D+TF) │
      └─────────────┘    └──────────────┘
```

---

## Bilinen Limitasyonlar (blocker degil)
- Sag gripper finger hareket etmiyor (park edildi — sol finger calisiyor, mock grip)
- Cotton boll 70mm vs Hand-E 50mm — boll fiziksel tutulmaz sim'de
- Sim ~%3-4 realtime — full cycle 5-10 dk wall clock surebilir
- Pre-grasp'da cluster full frame'e sigmayabilir — TODO: heuristic view adjustment ekle (hafif sag/sol/yukari/asagi kaydirarak tum boll'lerin frame icinde oldugunu dogrula, kenar boll'ler kesiliyorsa geri cekil veya pan yap)
- Pre-grasp view adjustment sirasinda cluster-level prediction kullanilmali (boll degil!). Once cluster bbox'in tamami frame icinde olsun, sonra boll-level detect + pick baslasin. Akis: approach → cluster predict → view adjust → cluster full frame'de → boll detect → pick

---

## Test Komutlari

### 1. Build (WSL2 icinde)
```bash
cd ~/harvesting_ws

# Interface'leri build et (yeni srv dosyalari)
colcon build --packages-select harvester_interfaces

# Orchestrator build
colcon build --packages-select orchestrator

# arm_commander build (yeni use_approach_orientation parametresi)
colcon build --packages-select robot_arm_moveit_config

# Source
source install/setup.bash
```

### 2. Launch (3 terminal, hepsi source'lu)

**Terminal 1 — Gazebo + controllers:**
```bash
source ~/harvesting_ws/install/setup.bash
ros2 launch robot_arm bot.launch.py
```
Beklenen: Gazebo acilir, robot + 3 cotton boll gorunur, controller'lar yuklenr.

**Terminal 2 — MoveIt + arm_commander + gripper:**
```bash
source ~/harvesting_ws/install/setup.bash
ros2 launch robot_arm_moveit_config moveit.launch.py
```
Beklenen: arm_commander HOME'a gider, "ARM COMMANDER READY" log'u, gripper_controller "ready" log'u.

**Terminal 3 — Pipeline (tum vision + orchestrator):**
```bash
source ~/harvesting_ws/install/setup.bash
ros2 launch orchestrator harvest_pipeline.launch.py
```
Beklenen: 7 node baslar (explorer, real_yolo_detector, depth_processor, camera_focus, spatial_detection_pipeline, harvest_executor, orchestrator_node). Her biri "ready" log'u basar.

### 3. Harvest Baslat (yeni terminal veya Terminal 3'te)
```bash
source ~/harvesting_ws/install/setup.bash
ros2 service call /orchestrator/start_harvest std_srvs/srv/Trigger "{}"
```

### 4. Izleme (ayri terminal'lerde)

**State degisimlerini izle:**
```bash
ros2 topic echo /orchestrator/status
```

**Ilerleme log'larini izle:**
```bash
ros2 topic echo /orchestrator/progress
```

**Scan durumunu izle:**
```bash
ros2 topic echo /explorer/scan_status
```

**Acil durdurma:**
```bash
ros2 service call /orchestrator/stop std_srvs/srv/Trigger "{}"
```

### 5. Birim Testleri (pipeline olmadan, tek tek test)

**Gripper test:**
```bash
ros2 service call /gripper/open std_srvs/srv/Trigger "{}"
ros2 service call /gripper/close std_srvs/srv/Trigger "{}"
```

**Arm test (home'a git):**
```bash
ros2 param set /arm_commander target_name home
ros2 service call /go_to_named std_srvs/srv/SetBool "{data: true}"
```

**Arm test (xyz'ye git):**
```bash
ros2 param set /arm_commander target_x 0.75
ros2 param set /arm_commander target_y 0.45
ros2 param set /arm_commander target_z 0.50
ros2 param set /arm_commander use_approach_orientation true
ros2 service call /go_to_pose std_srvs/srv/SetBool "{data: true}"
```

**YOLO test:**
```bash
ros2 service call /yolo/detect harvester_interfaces/srv/YoloDetect "{}"
ros2 service call /yolo/detect_clusters harvester_interfaces/srv/YoloDetect "{}"
```

**Depth test:**
```bash
ros2 service call /depth_processor/pixel_to_3d harvester_interfaces/srv/PixelTo3D "{u: 320, v: 240}"
```

**Tek boll pick test (harvest_executor):**
```bash
ros2 service call /harvest/pick_boll harvester_interfaces/srv/HarvestBoll "{boll_position: {x: 0.875, y: 0.475, z: 0.50}, pre_grasp_position: {x: 0.743, y: 0.403, z: 0.50}}"
```

**Sadece scan testi:**
```bash
ros2 service call /explorer/panoramic_scan std_srvs/srv/Trigger "{}"
# Bittikten sonra sonuclari al:
ros2 service call /detection/get_results harvester_interfaces/srv/GetDetectedClusters "{}"
```

---

## Durum Ozeti (2026-04-01)

### Tamamlanan
- [x] M1013 + Hand-E URDF (Gazebo Ignition uyumlu)
- [x] MoveIt2 config (SRDF, kinematics, collision matrix)
- [x] arm_commander.py (IK multi-seed + joint goal, pre-grasp offset, HOME fallback, approach orientation)
- [x] gripper_controller.py (open/close services, topic-based, sol finger calisiyor)
- [x] explorer.py (panoramic scan, arc sweep, detection pipeline entegrasyonu)
- [x] real_yolo_detector.py (YOLO inference, /yolo/detect + /yolo/detect_clusters)
- [x] spatial_detection_pipeline.py (YOLO -> focus -> depth -> 3D, cluster tracking, validation, /get_results)
- [x] camera_focus.py (pixel error -> joint adjustment)
- [x] depth_processor.py (pixel -> 3D world via TF)
- [x] Dynamic cotton bolls in world (3x static=false sphere, radius=0.035m)
- [x] harvest_executor.py (pick-and-place 8-step sequence, detayli loglama)
- [x] main.py (state machine: IDLE -> SCANNING -> APPROACHING -> HARVESTING -> RETURNING, full entegrasyon)
- [x] HarvestBoll.srv + GetDetectedClusters.srv interfaces
- [x] harvest_pipeline.launch.py (7 node, tek komut)
- [x] Code review + 4 bug fix (cluster ID mismatch, executor deadlock, double offset, kamera yonelimi)

### Bilinen Sorunlar (park edildi)
- Sag finger hareket etmiyor (controller claim ediyor, joint_states'te var, ama pozisyon degismiyor)
- Cotton boll 70mm cap vs Hand-E ~50mm max opening (boll sigmayabilir)
- Sim cok yavas (%3-4 realtime, camera 1 Hz)

---

## TASK 7: Demo Gorselleri + Video
**Amac:** Midterm sunumunda gosterilecek materyaller

**Yapilacak:**
- [ ] Her asamanin screenshot'lari (YOLO annotated images yolo_output/ klasorune kaydediliyor):
  1. HOME view -> scan basliyor
  2. Scan pozisyonlarinda YOLO detect sonuclari (cluster bbox'lar)
  3. Pre-grasp cluster view'e yaklasma
  4. Boll identification (bireysel boll detection)
  5. Pick cycle: approach -> grip -> lift -> reservoir
  6. Cluster bitti, home'a donus
- [ ] Gazebo'da full cycle video kaydi (ekran kaydedici)
- [ ] Alternatif: sim cok yavassa, her adimi ayri kaydet ve slide'lara koy

---

## TASK 8: Sunum Hazirlik
- [ ] Slide deck:
  - Mimari diagram (tum node'lar ve service baglantilari)
  - Eski 4-DOF vs yeni M1013 karsilastirma
  - YOLO model basarisi (ornek detection screenshot'lari)
  - Sim demo gorselleri/video
  - Hardware plani (lab visit notlari, ROBOCOB fotolar)
  - Roadmap: sim'den fiziksel robota gecis plani

---

## Notlar

### Scan Stratejisi
HOME'dan j1 rotate ile 3 pozisyon scan (robot yerinden kipirdamadan).
Ilerde (gercek robot): visual servoing ile dynamic pre-grasp view, surekli tahmin alarak yaklasma.

### Pick Cycle (sim demo, simdilik)
1. Config'den cluster pozisyonlari (veya scan ile detect)
2. Hardcoded pre-grasp offset (15cm)
3. YOLO + depth ile boll 3D pozisyonu (veya fallback: config)
4. Mock grip (arm hareket eder, gripper acar/kapar, ama boll fiziksel tutulmaz)
5. Sonuc: tam hareket sekansini gosteriyoruz
