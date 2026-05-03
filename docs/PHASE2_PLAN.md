# RoboCot Phase 2 — Mobile Orchard Harvesting (Sim-only)

**Date:** 2026-04-17
**Scope change:** Physical lab deployment iptal. Tum ilerleme simulation tarafinda. Husky mobile base + Doosan M1013 + reservoir, Clearpath orchard environment.

---

## 0. Progress Log

### Faz 1 — Environment Setup

| Task | Status | Output |
|------|--------|--------|
| **F1.1** Mesh port | ✅ DONE | `robot_arm/meshes/orchard/` — 3 .dae (orchard_world, orchard_trunks, orchard_leaves) + 5 textures (~111MB) |
| **F1.2** `orchard.world` (Ignition Fortress) | ✅ DONE | `robot_arm/worlds/orchard.world` — terrain physics + sky/sun + mesh visual-only (no trunk/leaf collision) |
| **F1.3** Tree positions YAML | ✅ DONE | `robot_arm/config/orchard_tree_positions.yaml` — 236 trees, 11 rows, bbox X[4,41] Y[3,35], canopy Z[0.4,2.0]m. Extracted via `robot_arm/scripts/extract_tree_positions.py` (voxel-cluster on low-Z trunk vertices). |
| **Sanity #1** M1013 in orchard.world | ✅ PASS | `orchard_test.launch.py` — standalone arm spawns at (0,0), TF saglikli, 3 controller aktif, mesh renderlendi |
| **F1.4** Husky URDF + mount + arm + reservoir | ✅ DONE | `robot_arm/urdf/husky_robocot.urdf.xacro` + `m1013_robocot.urdf.xacro` refactored with `standalone` xacro arg. Husky body 0.99×0.67×0.30m, 4 wheels (r=0.165m), arm mount @ x=+0.20 (front-center), reservoir walled box @ x=-0.30 (rear). |
| **Sanity #2** Husky composition spawn | ✅ PASS | All 23 link segments loaded in robot_state_publisher; world_to_husky fixed joint skipped by gz_ros2_control as expected; controllers all green. |
| **Bonus** WASD teleop drivability | ✅ DONE | `husky_robocot.urdf.xacro` `mobile=true` arg → DiffDrive plugin (skid-steer, 4 wheels). `husky_gz_bridge.yaml` (cmd_vel ROS→GZ, odom GZ→ROS, tf GZ→ROS). `static_tf_world_to_odom` (identity, for orchestrator "world" frame compatibility). `orchestrator/wasd_teleop` node (w/s/a/d/q/e/z/c/+/-). Confirmed working. |
| **F1.5** MoveIt config update | ⏳ TODO | SRDF: yeni named pose'lar (scout_left/right, harvest_ready). arm_commander D1 fix (j1 validation gevset). |
| **F1.6** Reach validation | ⏳ TODO | Hardcoded test pozisyonlari, IK + canopy reach |
| **F1.7 v1** Boll spawner (static) | ✅ DONE | `scripts/generate_bolls.py` → 1416 boll (236 tree × 5-7), 70/30 ripe/unripe. `config/orchard_bolls.yaml` (ground truth) + `worlds/orchard_bolls.world` (auto-gen). Static visual-only. |
| **F1.7 v2** Active-radius dynamic toggling | ⏳ NEXT | `boll_spawner.py` ROS node, robot_pose → 3-5m yariçap → static↔dynamic toggle. F1.8'e prerequisite. |
| **F1.8** Vehicle-independent pick test | ⏳ TODO | D2 fix (gripper bypass kaldir), 8-step pick on real boll |

### Bilinen Notlar / Park Edilenler
- **KDL warning** (`root link husky_base_link has inertia, KDL doesn't support`) — non-critical; F1.5 MoveIt entegrasyonunda problem yaratirsa `base_footprint` dummy link eklenir.
- **Deprecated `ign_ros2_control`** — Gazebo Fortress versiyonu, non-blocking warning. Yeni `gz_ros2_control` adi gelecek release'lerde gerekecek.
- **Controller update period 0.01s vs sim 0.005s** — performans warning, sim hizini etkilemiyor.

---

## 1. Yeni Sistem Tanimi

```
[ Husky mobile base ]
   |-- deck (0.39m yuksek)
   |   |-- mount plate (~0.10m) -> M1013 base @ ~0.5m
   |   |-- Reservoir (open box, sinirsiz capacity)
   |
   +-- Wheels, IMU, base nav stack (Nav2)

Wrist camera: M1013 son link (mevcut RGB-D, eye-in-hand)
   -> hem cluster detection hem boll-level detection bu kameradan gelir
```

**Mission cycle:**
```
START -> navigate (kol pre-defined "scout" pozisyonunda, agac satirina bakar)
   -> wrist cam YOLO -> cluster bbox tespit
   -> if cluster yeterli confidence: STOP base
   -> view-point hesapla (cluster trunk pozisyonuna ortho 0.7m)
   -> drive to view-point
   -> harvest cycle (mevcut 8-step pick) x N boll
   -> reservoir drop on each pick
   -> wrist cam tekrar tara: hala boll var mi?
        Evet -> harvest cycle devam
        Hayir -> NAVIGATING'e don
   -> next cluster
   -> END (route biterse)
```

**Onemli:** Forward-looking ek kamera **yok**. Tum perception wrist kamera uzerinden:
- Navigation sirasinda kol "scout pose"da sabit dururdu (sag/sol yana bakarak agac satirini gorur)
- Cluster detection -> stop -> view-point movement -> harvest cycle (kol harvest pose'una gecer)
- Harvest bitince scout pose'a don, navigation devam

---

## 2. Orchard Bulgulari (extract_trees_v2.py'den)

| Parametre | Deger |
|-----------|-------|
| Toplam agac | **219** |
| Satir sayisi | **11** |
| Plant edilmis alan | **~38m × 32m** |
| Satir-ici X spacing | **median 1.50m**, mean 1.65m, std 0.67m |
| Satirlar arasi gap | **~3m** (Husky icin rahat) |
| Trunk Z range | 0 -> 1.5m |
| Canopy yaklasik | 1.5 -> 3-4m |
| Husky boyut | 0.99 × 1.39 × 0.39m, 75kg payload |
| M1013 spec | 33kg, 1.3m reach, 6-DOF |

**Reach analizi:**
- Husky deck 0.39m + mount 0.10m -> M1013 base @ 0.50m
- M1013 reach 1.3m -> tcp ~1.8m yuksege erisir (base @ 0.5m + 1.3m vertical)
- Canopy lower-mid (1.5-2.5m) bizim sweet spot. Ust canopy (3m+) erisilmez — bu gercek constraint, problem degil.

**Mesh notu:** Tum agaclar tek `.dae` icinde bake edilmis, per-tree query yok. 219 pozisyonu kendi `tree_positions.yaml`'a dump edip kullaniyoruz.

---

## 3. Confirmed Decisions

| # | Karar | Detay |
|---|-------|-------|
| 1 | Mobile base = Husky | Hazir model, ROS2 destegi var. Doosan + reservoir uzerine direkt entegre. Doosan **scale edilmez** — original size kalir, mount plate ile yukseklik ayarlanir. |
| 2 | Cluster decision | Iki cluster gorulurse leftmost'a focus |
| 3 | Boll fizigi | Real grip: gravity + collision + grip aktif. **Agaclar collision-disabled** (sadece visual) — robot agaclara carpsa bile gecer. |
| 4 | Reservoir | Sinirsiz capacity, full-check yok |
| 5 | Feature map interpretation | Sonraki is, simdilik scope disinda |
| 6 | RL/IL | Yok |

---

## 4. ROS1 -> ROS2 Mesh Portu

`cpr_orchard_gazebo` Noetic + Gazebo Classic. Biz Humble + Ignition.
**Plan:** Sadece 3 mesh dosyasini cek, Ignition world'umuze ekle:

| Source | Hedef |
|--------|-------|
| `cpr_orchard_gazebo/meshes/orchard_world.dae` (terrain) | `robot_arm/meshes/orchard/orchard_world.dae` |
| `orchard_trunks.dae` | `robot_arm/meshes/orchard/orchard_trunks.dae` |
| `orchard_leaves.dae` | `robot_arm/meshes/orchard/orchard_leaves.dae` |

`worlds/cotton_field.world` -> `worlds/orchard.world`. Tek static link, 3 visual mesh, **collision sadece terrain icin** (trunks + leaves collision yok).

---

## 5. Phase Breakdown

### Faz 1 — Environment Setup (~1.5-2 hafta)

| # | Task | Detay |
|---|------|-------|
| F1.1 | Mesh port: 3 .dae bizim meshes/orchard/'a | `cpr_orchard_gazebo/meshes/*` -> `robot_arm/meshes/orchard/` |
| F1.2 | `orchard.world` (Ignition) | Empty world + spawn orchard URDF, gun ışıgı + sky |
| F1.3 | Tree positions config | `extract_trees_v2.py` -> `config/tree_positions.yaml` (219 entry: id, x, y) |
| F1.4 | Husky URDF + mount plate + M1013 + reservoir | Husky URDF'i fork et, plate (rectangular box, 0.10m) + M1013 chain + reservoir box ekle |
| F1.5 | MoveIt2 config update (yeni base frame, kinematic chain Husky deck'ten) | base_link -> husky_base, M1013 chain husky_top_plate'den baslar |
| F1.6 | Reach validation (RViz IK testi, lower canopy reachability) | Hardcoded test pozisyonlari ile arm_commander cagrilari |
| F1.7 | Boll spawner script | Her agac icin random N (5-10) sphere, ripe (beyaz) %70 + unripe (yesil/kahve) %30. Lower canopy 1.5-2.5m yukseklikte konumlandir. Dynamic objects (gravity + collision). |
| F1.8 | Vehicle-independent pick cycle test | Husky'yi sabit konuma koy (1 agac karsisinda), mevcut 8-step pick'i runla. Sphere collision check, grip success rate olc. |

### Faz 2 — Mobile Base Integration (~2 hafta)

| # | Task | Detay |
|---|------|-------|
| F2.1 | Nav2 stack + orchard | Husky Nav2 default config'i kullan. Static map: orchard'in 2D occupancy grid'i (manual cizilebilir veya gmapping ile bir kerelik run). |
| F2.2 | Scout pose tanimi | Kol pre-defined "scout" joint config'i: kamera satir yonune dik bakar (sol satir tarafindan ilerlerken sol yana). Navigation boyunca kol bu pose'da sabit. |
| F2.3 | Cluster detection on the move | Yeni node: `cluster_navigator`. **Wrist cam** YOLO inference, navigation hizinda surekli calisir. Cluster bbox + size-based distance estimate. Confidence threshold gecerse "DUR" command. |
| F2.4 | View-point computation | Cluster center'a (kabaca agac trunk pozisyonu) ortho yaklasim: 0.7m mesafe, robot heading agacin perpendiculari. |
| F2.5 | Pre-defined route | 11 satir + zigzag pattern. Route Nav2 waypoint listesi olarak yaml'a yazilir. |
| F2.6 | Orchestrator FSM extension | Mevcut state machine: IDLE -> SCANNING -> APPROACHING -> HARVESTING -> RETURNING. Yeni: IDLE -> NAVIGATING(scout pose) -> DETECTING -> STOPPING -> APPROACHING_CLUSTER -> HARVESTING -> RESUMING_NAV(scout pose). |
| F2.7 | "Bitti" karari | HARVESTING bitince /yolo/detect tekrar -> hala boll var mi? Varsa devam, yoksa NAVIGATING'e dön. |
| F2.8 | Mobile base + arm coordination | Base hareket ederken kol scout pose'unda kilitli (joint goal hold). Stop sonrasi kol harvest moduna gecer. Base + arm ayni anda hareket etmez (basitlik). |

### Faz 3 — ML / Dataset (paralel) (~2-3 hafta)

| # | Task | Detay |
|---|------|-------|
| F3.1 | Real cotton dataset (web/benchmarks) | Kaggle/Roboflow + scrape, 1500-2500 imaj |
| F3.2 | Blender renders (orchard.blend mevcut!) | 1000-2000 imaj, varied lighting + camera angles + boll counts |
| F3.3 | Augmentation pipeline | albumentations: brightness, hue, blur, partial occlusion, motion blur (mobile cam) |
| F3.4 | YOLO11 retrain | classes: cotton_boll, cotton_cluster, unripe_boll, partial_occluded |
| F3.5 | Validation set + KPI'lar | mAP@50, mAP@50-95, per-class AP, false positive rate |
| F3.6 | (Sonraki is) Grad-CAM feature interpretation | Layer activations + saliency maps. Rapor icin parlak deliverable. |

### Faz 4 — Telemetry + App (~1 hafta)

| # | Task | Detay |
|---|------|-------|
| F4.1 | Telemetry topics genisletme | `/orchestrator/status` + `/husky/odom` + `/yolo/conf` + `/harvest/count` |
| F4.2 | Live dashboard | Remotion proje var. Web-based realtime panele cevir (rosbridge_server + react front). Map + state + counters. |
| F4.3 | rosbag2 logging + replay | Tum demo run'i logla, parser ile rapor metrikleri cikar |

### Faz 5 — Cleanup + Docs + Demo (~1 hafta)

| # | Task | Detay |
|---|------|-------|
| F5.1 | Monorepo cleanup | _legacy/ sil, package'lari topla |
| F5.2 | README + setup docs | Quick start, dependencies, launch komutlari |
| F5.3 | Demo video (full cycle) | Husky 2-3 cluster harvest, full pipeline gorunsun |
| F5.4 | Final sunum | Slide deck, key metrics, future work |

---

## 6. Acik Sorular (her faz icin)

### F1
- Husky URDF source: `clearpath_simulator` mu, manuel kompoze mi? Stock Husky modeli mevcut, fork hizli.
- Reservoir gorsel boyutu: 0.4×0.4×0.3m (Husky deck'in 1/3'unu kapliyor) yeterli mi?
- Boll sayisi: agac basina 5-10 random mi sabit 7 mi? Demo icin sabit daha okunur.
- Boll material/visual: pure white sphere mi, biraz texture (cotton fluffy) mi? Sphere yeterli, sonra render edilebilir.

### F2
- Scout pose'un joint configurasyonu: yumusak transition icin home-pose'a yakin, ama kamera satir yonunde 90° donuk olmali. Hangi joint kombosu? F1.6 reach validation sirasinda belirlenecek.
- Wrist cam'in motion blur'u: Husky hareket halinde kamera 1Hz update yetersiz olabilir. Sim camera rate'ini 5-10Hz'e cikarmak gerek.
- Cluster detection mesafesi: wrist cam HFOV 90°, base 0.5m yukseklikte, kol ekstra 0.3-0.5m yukari verirse kamera 0.8-1.0m yuksekligindeki canopy'yi 1-3m mesafeden gormeli. Detection size threshold buna gore kalibre.
- Route ne kadar uzun: 11 satir tam mi yoksa 2-3 satir demo icin yeterli mi? Demo icin 2-3 satir, full run rapor icin.
- Cluster siralamasi: leftmost-first kuralini iki cluster ayni anda gorulurse mu uyguluyoruz, yoksa global priority var mi? Sadece "iki cluster ayni anda" durumda.

### F3
- Class set: 3 mi (boll, cluster, unripe), 4 mu (occluded ekle)? 4 onerilir, mobile cam'da occlusion sik.
- Test set ground truth: simulation'dan synthetic GT mi (kolay), real'den manual annotation mi (zaman) ?

### F4
- Live dashboard tamamen yeni mi yoksa Remotion'a real-time eklenecek mi? Remotion ofline render'a dayali, real-time icin `web_video_server` + react daha mantikli.

---

## 7. Riskler

| Risk | Etki | Mitigation |
|------|------|------------|
| Mesh portu sirasinda DAE Ignition'da renderlanmama | High (envman butun gecmesi) | Once tek agac mesh test, sonra full orchard. |
| Husky deck uzerinde M1013 stabilite (CoG yuksek) | Medium (sim'de devirme) | Husky base mass'ini artir, suspension ayari. Sim only — gercek sorun degil. |
| Nav2 + tree collision (devre disi) confused | Low | Static occupancy map kullan, dinamik kollision kapali. |
| Boll spawn density (cok = laggy, az = boring) | Medium | 5-7 boll/agac civari, 219 agac × 6 = ~1300 dynamic obj — Gazebo'yu sleyebilir. **Onlem:** sadece robot yakinindaki ~3 agacin bollarini "active" tut, gerisini static. |
| YOLO retrain dataset cok kucuk -> overfitting | Medium | Augmentation cok, transfer learning, validation set | 
| Tree mesh bake edildi -> tek tek interaction yok | Low (zaten visual-only istiyoruz) | Tree positions config dump'tan alinir |

---

## 8. Hemen Yapilacak (Kickoff)

1. F1.1 — Mesh dosyalari kopyala
2. F1.3 — Tree positions YAML cikar (extract_trees_v2.py refine)
3. F1.2 — Minimal `orchard.world` (ust uste 3 mesh + sun + sky)
4. Sanity test: mevcut M1013 sabit base ile orchard icinde spawn olabiliyor mu? (Husky henuz yok)

Bu 1.5 isgununde biter, F1 jumpstart icin yeterli.
