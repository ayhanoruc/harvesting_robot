# RoboCot Phase 2 — Running Notes & Specifications

**Created:** 2026-04-17
**Purpose:** Tum measurement, calc, decision ve onging note'larin merkezi. PHASE2_PLAN.md plan/strateji dokumani; bu dosya ise spec/data dokumani — yapilacaklar degil, **yapildigi/karar verildigi bilgiler** burada birikir.

---

## A. Environment Measurements (Ground Truth)

Kaynak: `cpr_orchard_gazebo/meshes/{orchard_world,orchard_trunks,orchard_leaves}.dae` mesh'lerinden vertex parse + voxel cluster.

### A.1 Orchard Footprint

| Parametre | Deger | Birim | Kaynak |
|-----------|-------|-------|--------|
| Tree count | 236 | adet | extract_tree_positions.py |
| Row count | 11 | satir | Y-axis gap detection (>1.5m) |
| Average trees per row | ~21 | adet | 236 / 11 |
| Plant edilmis bbox X | 4.0 → 41.0 | m | mesh vertex bbox |
| Plant edilmis bbox Y | 3.0 → 35.0 | m | mesh vertex bbox |
| Plant edilmis alan | 37 × 32 | m² | bbox span |
| Within-row X spacing (median) | 1.50 | m | extract analiz |
| Within-row X spacing (mean) | 1.65 | m | extract analiz |
| Within-row X spacing (std) | 0.67 | m | extract analiz |
| Inter-row Y gap | ~3.0 | m | gozle (tahmini, agacliklar arasi yol) |
| **Margin alanlari** | ~5m her tarafta | m | mesh ground extent vs trees |

### A.2 Tree Geometry (Per-Tree)

| Parametre | Deger | Birim | Not |
|-----------|-------|-------|-----|
| Trunk Z range | 0 → 1.5 | m | dao mesh local frame |
| Canopy Z range (toplam) | 0.4 → 2.0 | m | leaves mesh bbox |
| Canopy Z range (per-tree mean) | ~0.7 → ~1.85 | m | search radius=0.8m, leaf vertex avg |
| Canopy radius (yatay) | ~0.5-0.7 | m | tahmin (gorsel uzerinden) |
| Trunk thickness | ~5-10 | cm | gorsel tahmin |

### A.3 Coordinate Frames

```
World frame: Gazebo'nun world origin (0,0,0)
  Orchard mesh sw kosesi @ (~3, ~3) — rotation YOK (raw mesh frame)
  Tree positions YAML world-frame coordinatelarinda dogrudan kullanilabilir
```

---

## B. Robot Stack Dimensions

### B.1 Husky Body

| Parametre | Deger | Birim |
|-----------|-------|-------|
| Length × Width × Body Height | 0.99 × 0.67 × 0.30 | m |
| Body Z (bottom → top) | 0.13 → 0.43 | m (deck top z=0.43) |
| Wheel radius | 0.165 | m |
| Wheel width | 0.10 | m |
| Wheel track (left↔right separation) | 0.555 | m |
| Wheel base (front↔rear separation) | 0.51 | m |
| Mass (body) | 50 | kg |
| Mass (per wheel) | 2 | kg |

### B.2 Arm Mount + Arm

| Parametre | Deger | Birim |
|-----------|-------|-------|
| Mount plate dimensions | 0.20 × 0.20 × 0.05 | m |
| Mount position on deck | x=+0.20 (ön-merkez), y=0, z=+0.025 | m |
| **M1013 base (base_0) world Z** | 0.43 + 0.05 = **0.48** | m |
| **M1013 joint1 origin world Z** | 0.48 + 0.1525 = **0.633** | m |
| M1013 horizontal reach (specified) | 1.296 | m |
| M1013 max vertical reach (above j1) | ~1.20 | m (estimate) |
| M1013 max payload | 10 | kg |
| TCP offset from tool0 | 0.14 | m (along Z) |

### B.3 Reservoir

| Parametre | Deger | Birim |
|-----------|-------|-------|
| Outer dimensions | 0.40 × 0.40 × 0.20 | m |
| Wall thickness | 0.01 | m |
| Position on deck | x=-0.30 (arka), y=0, z=+0.10 | m (top edge @ world z=0.63m) |
| Capacity | unlimited (no full check) | - |

---

## C. Reach Geometry — Analytical

### C.1 M1013 Workspace (Husky Mounted)

Base center: world (xR, yR, 0.48m)  where (xR, yR) is robot base footprint center.

Reach envelope (yaklasik): merkez = joint1 origin = (xR, yR, 0.633), yariçap = 1.296m + 0.14m TCP = ~1.44m sphere.

Bu sphere icindeki noktalar **kinematic olarak** erisilebilir (collisions ignored). Ama gercek reach IK + joint limits ile sinirli:
- joint1 base rotation: ±360° (+ scout pose'lar icin -135°/+135° validation by arm_commander, **D1 fix** sonrasi gevsetilecek)
- joint3 elbow: ±160° (sinir burda)

### C.2 Canopy Reachability Test (Geometrik)

Robot Husky uzerinde **tree center'a 0.7m mesafede dururken**:

| Hedef Z (canopy) | Distance from j1 origin | In reach? |
|------------------|------------------------|-----------|
| 0.5m  (alt canopy) | sqrt(0.7² + (0.5-0.633)²) = 0.71m | ✅ Easy |
| 1.0m  (orta canopy) | sqrt(0.7² + (1.0-0.633)²) = 0.79m | ✅ Easy |
| 1.5m  (üst-orta canopy) | sqrt(0.7² + (1.5-0.633)²) = 1.11m | ✅ OK |
| 2.0m  (canopy peak) | sqrt(0.7² + (2.0-0.633)²) = 1.54m | ⚠️ Borderline (>1.44m envelope) |

**Sonuc:** Ust canopy peak'i (~2.0m) Husky 0.7m mesafedeyken arm reach'in **biraz uzeri**. Pratikte:
- Robot 0.5m mesafeye yaklassin → hedef-base distance dusur
- Veya alt-mid canopy'de boll spawn et (max 1.7m), peak'i kullanma
- F1.6 reach validation testi netlestirir

### C.3 Reservoir Drop Reach

Reservoir top edge: world z = 0.43 + 0.20 = **0.63m** (mount level), x = -0.30 (Husky body local).

Robot kendi uzerindeki reservoir'a ulasmak icin: TCP reservoir x=-0.30 + reservoir_lx/2 = -0.10'a kadar uzanmali. Mount (arm base) x=+0.20, fark 0.30m.

| Reservoir hedef | TCP'ye mesafe (j1 origin'den) | In reach? |
|-----------------|------------------------------|-----------|
| Reservoir center top: x=-0.30, z=0.63 | sqrt((0.30+0.20)² + (0.63-0.633)²) ≈ 0.50m | ✅ Easy (drop hover) |

Yani drop pose icin reach problem yok.

### C.4 Reach Calc Source Code

`extract_tree_positions.py` benzeri bir reach validator script F1.6'da yazilacak. RViz'de IK call'lar ile dogrulama yapilir.

---

## D. Cluster Strategy

### D.1 Cluster Definition

**Karar:** Her agac = 1 cluster. 236 cluster total.

| Parametre | Deger |
|-----------|-------|
| Cluster center | tree (x, y) — orchard_tree_positions.yaml'dan |
| Cluster volume | ~50cm radius × canopy Z range |
| Bolls per cluster | random 5-7 (F1.7'de spawner ile) |
| Boll dagilimi | trunk x,y +/- random 0-30cm offset, z within canopy_z_min..max |
| Ripe : Unripe ratio | 70% : 30% |
| Ripe boll | beyaz sphere r=0.035m |
| Unripe boll | yesil/kahve sphere r=0.025m |

### D.2 Cluster Detection Logic

Wrist cam scout pose'da satira dik bakar:
- Far view: cluster bbox merkezi visible → cluster ID + tree pozisyonu (config'den match)
- Iki cluster ayni anda gorulurse: **leftmost'a focus** (user karari)

### D.3 Active-Radius Optimization

Performans icin:
- Robot pozisyonuna gore 3-5m yaricapinda bolls `<static>false</static>` (dinamik, gravity + collision)
- Disindaki tum bolls `<static>true</static>` (visual-only, fizik yok)
- Robot hareket edince `boll_spawner` arkaplan thread'i state'i guncelles
- Hedef: 1500 boll'un sadece ~50 tanesi ayni anda dinamik → CPU bujet rahat

---

## E. Navigation Path Design

### E.1 Start Pozisyonu

```yaml
start_pose:
  x: 3.0
  y: 5.0    # row 0'in onunde, SW kose
  yaw: 0.0  # +X yonune bakar (orchard'a dogru)
```

Mevcut `husky_test.launch.py` bu pozisyonda spawn'liyor.

### E.2 Route Topolojisi (Onerilen)

11 satir × ~21 agac = 236 stop. Demo icin 2-3 satir yeterli.

**Zigzag/snake pattern** (Y eksen sabit corridor, X yon degisken):

```
Row 0   ──→──→──→──→ ...    (Y=4'den, X 4→41 git)
                  │
                  ▼
Row 1   ←──←──←──← ...      (Y=7'den, X 41→4 dön)
                  │
                  ▼
Row 2   ──→──→──→──→ ...    (Y=10'dan, X 4→41 git)
...
```

### E.3 Per-Cluster Stopping Logic

Robot satir boyunca giderken kol **scout pose**'da satira dik bakar (örn. row 0'da row 0'a baksin: Y=4 satirinda, kol +Y veya -Y yonune dogru bakar — sol yan tarafa):

| Position | Husky pose | Scout pose hedef |
|----------|------------|------------------|
| Row 0 corridor | Y=5.5 (row 0 ile row 1 arasi) | Kameranin `+Y` yonune dogru bakmasi → row 0 agaclarina (Y=4) |
| Row 0 → Row 1 dönüş | Y=8.5 (row 1 ile row 2 arasi) | Kameranin `-Y` yonune dogru bakmasi → row 1 agaclarina (Y=7) |

**Cluster detection on the move:**
1. Husky surekli ileri (Düşük hizla, ~0.2 m/s)
2. Wrist cam YOLO inference (5 Hz desen)
3. Cluster bbox confidence > threshold → STOP
4. Cluster trunk pos hesapla (Husky odom + cam relative)
5. View-point hesapla: trunk'a 0.7m mesafede ortho approach
6. Husky → view-point (kucuk lokal hareket)
7. Harvest cycle (8-step pick × N boll)
8. Resume nav: scout pose'a dön, ileri devam

### E.4 Demo Subset

Full 11-row × 21-tree = 236 cluster ÇOK uzun (her cluster ~5-7 boll × 8-step pick = ~30 sn → 4 saat full demo).

**Demo akisi:** sadece **row 0'da ilk 3 cluster** (3 stop, ~3-5 dk).

---

## F. Pre-defined Checks (F1 Sonu)

Bu check'leri F1.6 ve F1.8'de kod olarak yaziyoruz:

| Check | Kriterler | Status |
|-------|-----------|--------|
| **C1** Husky orchard'da spawn olur | husky_test.launch hata vermeden acar, robot duruyor | ✅ PASS |
| **C2** Tum URDF link'leri TF'te | 23 segment, world→...→tcp tam zincir | ✅ PASS |
| **C3** Husky drivable | WASD ile cmd_vel publish → odom guncellenir → robot hareket eder | ✅ PASS |
| **C4** Wrist cam image topic geliyor | `/camera/color/image_raw` 30Hz | TODO (test komutu) |
| **C5** Wrist cam depth image | `/camera/depth/image_raw` valid | TODO |
| **C6** TF: world → camera_optical_frame chain valid | depth_processor.lookup basarili | TODO |
| **C7** IK: robot 1.5m height target reach | arm_commander 1.5m'de IK basarili | F1.6 |
| **C8** IK: 2.0m peak target reach | borderline, expected fail at full reach | F1.6 |
| **C9** Reservoir drop pose IK | TCP reservoir top'a ulasir | F1.6 |
| **C10** Boll spawn density runtime impact | RTF (real-time factor) >= 0.5 with 1500 bolls (active radius mode) | F1.7 |
| **C11** Pick cycle on real boll | 8-step pick: gripper close, lift, drop in reservoir | F1.8 |
| **C12** Boll fizigi: gravity + grip | Open gripper → boll dustur, close → boll yapisir | F1.7/F1.8 |

---

## G. Decisions Made (Log)

| # | Tarih | Karar | Gerekce |
|---|-------|-------|---------|
| 1 | 2026-04-17 | Husky base scale = original (0.99×0.67) | Orchard 3m corridor'a rahatca sigar, scale gerekmedi |
| 2 | 2026-04-17 | M1013 base scale = original | Reach analizi gosterdi, alt-mid canopy reachable |
| 3 | 2026-04-17 | Mount plate = 5cm minimal | Canopy alcak (max 2.0m), büyük plate gereksiz |
| 4 | 2026-04-17 | Reservoir front-back orientation: arm front, reservoir rear | Mantikli (arm cluster'a erisir, drop kendi arkadan) |
| 5 | 2026-04-17 | Tree count = 236 (extract result), tasks_today's 219 sayisi guncellendi | Mesh-extracted ground truth |
| 6 | 2026-04-17 | Trunk + leaves collision **OFF** | Robot agaclara carpsa bile gecer (visual-only) |
| 7 | 2026-04-17 | Active-radius boll spawn (3-5m) | Performans, lokal CPU/GPU sinirli |
| 8 | 2026-04-17 | Each tree = 1 cluster, leftmost preference | Sik dikim sebebi, basit semantik |
| 9 | 2026-04-17 | Wrist cam tek kamera, forward cam YOK | Tek source, kol scout pose ile cluster detection |
| 10 | 2026-04-17 | Demo subset: row 0 ilk 3 cluster | Full 236 cluster cok uzun |
| 11 | 2026-04-17 | DiffDrive plugin (skid-steer 4-wheel) | F2 Nav2 entegrasyonu icin standart pattern |
| 12 | 2026-04-17 | Cluster yaklasim mesafesi: **0.5m** (0.7m yerine) | Reach analizi gosterdi: 0.7m'de canopy peak (2.0m) borderline, 0.5m'de tum canopy reachable |
| 13 | 2026-05-03 | **Plan B1 bolls:** static **visual-only** spheres + **mock teleport** | Ignition+DART'ta dynamic boll yerel stabilite yok; goruntu dis yuzeyde, pick Gazebo `set_pose` ile |

---

## H. Open Questions / TODO Refinements

1. **Scout pose joint config:** F1.5/F1.6'da tespit edilecek. j1 +90° veya -90°'ye gidip kameranin satira dik baktigi pozisyon. arm_commander D1 fix gerekli (j1>135° rejection gevsetilmeli).
2. **Reach borderline (peak 2.0m):** Bolls'lari max 1.7m'ye kadar spawnlamak mi, yoksa Husky daha yakina (0.5m) park mi? F1.6'da test.
3. **DiffDrive max velocity:** 1.0 m/s linear, 1.5 rad/s angular (URDF'te). Cluster detection on the move icin 0.2-0.3 m/s optimal mi? F2'de tune.
4. **MoveIt planning frame:** "world" mu "odom" mu? Husky hareket edince base_0 da hareket eder, MoveIt planning scene world frame'de tanimli. Sim'de world==odom (static identity) sayesinde sorun yok.
5. **rosgz_bridge tf_topic:** DiffDrive plugin `/tf` topic'ine yaziyor. `parameter_bridge` GZ→ROS bridge ile uyusuyor — ama ros2'de tf message'i `/tf` topic'i (latched mi degil mi) kontrol et.
6. **Ground texture render:** orchard_world.dae visual gosteriyor ama `<static>true</static>` ground_plane'in collision'i var. Her ikisi cakisirsa visual sorun olabilir. Gorsel kontrol gerekli.

---

## I. Changelog (Running)

| Tarih | Sub-task | Detay |
|-------|----------|-------|
| 2026-04-17 | F1.1 | 3 .dae + 5 texture kopyalandi `meshes/orchard/`'a |
| 2026-04-17 | F1.2 | `orchard.world` yazildi, mesh visual-only spawn, sky+sun+terrain physics |
| 2026-04-17 | F1.3 | `extract_tree_positions.py` yazildi, `orchard_tree_positions.yaml` cikarildi (236 trees, 11 rows) |
| 2026-04-17 | Sanity #1 | M1013 standalone orchard.world'de spawn ve render OK |
| 2026-04-17 | F1.4 | `husky_robocot.urdf.xacro` + m1013 standalone arg refactor |
| 2026-04-17 | Sanity #2 | Husky composition 23 segment OK |
| 2026-04-17 | Bonus | DiffDrive + WASD teleop entegrasyonu, husky drivable |
| 2026-04-17 | Doc | PHASE2_NOTES.md (this file) created |
| 2026-04-17 | F1.7 v1 | `generate_bolls.py` yazildi. 1416 boll (236 tree × 5-7) seeded random uretildi: 987 ripe (70%) + 429 unripe (30%). `orchard_bolls.yaml` (ground truth) + `orchard_bolls.world` (orchard.world + 1416 static boll models) generate edildi. husky_test.launch.py orchard_bolls.world'u tercih ediyor. |
| 2026-05-03 | Plan B1 bolls | **Static visual-only** spheres (collision yok): satir bazli **koridor yüzü** (+/-Y kabuk normali), mesafe ~0.22–0.42 m, Z üst-canopy biased. **`harvest_executor`:** `mock_gazebo_teleport` ile Gazebo `set_pose`; `_call_gripper` bypass dışında gerçek gripper çağrısı düzeltildi. Zayıf makine için: `generate_bolls.py --num-trees N` |

(Yeni adimlar geldiginde bu tabloya ekle.)

| 2026-05-03 | B1 refine | Boll placement generator yeniden ayarlandi: canopy outer-shell ve gorunurluk odakli (ground'a dusmeyen Z clamp). harvest_executor mock-teleport akisi iyilestirildi: nearest boll seciminde picked olanlar tekrar secilmiyor, release sonrasi reservoir icine grid-stacked drop uygulan�yor. |
