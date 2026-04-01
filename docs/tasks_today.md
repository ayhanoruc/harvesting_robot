# Midterm Sunumuna Kadar TODO

## Durum Ozeti (2026-04-01)

### Tamamlanan
- [x] M1013 + Hand-E URDF (Gazebo Ignition uyumlu)
- [x] MoveIt2 config (SRDF, kinematics, collision matrix)
- [x] arm_commander.py (IK multi-seed + joint goal, pre-grasp offset, HOME fallback)
- [x] gripper_controller.py (open/close services, topic-based, sol finger calisiyor)
- [x] explorer.py (panoramic scan 21 poz, arc sweep, detection pipeline entegrasyonu)
- [x] real_yolo_detector.py (YOLO inference, /yolo/detect + /yolo/detect_clusters)
- [x] spatial_detection_pipeline.py (YOLO -> focus -> depth -> 3D, cluster tracking, validation)
- [x] camera_focus.py (pixel error -> joint adjustment)
- [x] depth_processor.py (pixel -> 3D world via TF)
- [x] Dynamic cotton bolls in world (3x static=false sphere, radius=0.035m)
- [x] harvest_executor.py SKELETON (pick-and-place 8-step sequence)
- [x] main.py SKELETON (state machine: IDLE -> SCANNING -> APPROACHING -> HARVESTING -> RETURNING)
- [x] HarvestBoll.srv interface

### Bilinen Sorunlar (park edildi)
- Sag finger hareket etmiyor (controller claim ediyor, joint_states'te var, ama pozisyon degismiyor)
- Cotton boll 70mm cap vs Hand-E ~50mm max opening (boll sigmayabilir)
- Sim cok yavas (%3-4 realtime, camera 1 Hz)

---

## TASK 1: Scan Basitlestirme
**Amac:** HOME'da durarak sadece j1 (pan) ile kamerayi cevirip tum clusterlari tara

**FOV Analizi:**
- Cluster angular span = 57 derece (cluster_3: -28.5 derece, cluster_2: 0 derece, cluster_1: +28.5 derece)
- Camera HFOV = 90 derece -> tek frame'e 3'u de sigar (16.5 derece margin)
- Vertical: clusterlar kameradan ~15-18 derece asagida, 74 derece VFOV ile rahat sigar

**Yapilacak:**
- [ ] explorer.py'deki panoramic scan parametrelerini override et:
  - 3 pan pozisyon: j1 = [-0.50, 0.0, +0.50] rad (-28.5, 0, +28.5 derece)
  - j2/j3/j4/j5/j6 HOME'da sabit
  - Her cluster en az 2 pozisyondan gorunur (detection merge icin)
- [ ] Opsiyonel: j5 ile hafif tilt ekle (1 ek satir = 6 poz total) — gerekirse
- [ ] Test: scan baslat, her pozisyonda YOLO detect cagir, 3 cluster tespit edilsin

---

## TASK 2: Pipeline Launch File
**Amac:** Tek komutla tum pipeline node'larini baslat (simdi hepsi manuel)

**Yapilacak:**
- [ ] `harvest_pipeline.launch.py` olustur (moveit.launch.py'ye ekle veya ayri dosya):
  - explorer
  - real_yolo_detector
  - depth_processor
  - camera_focus
  - spatial_detection_pipeline
  - harvest_executor
  - orchestrator_node (main.py)
- [ ] Tum node'lara config_file parametresi gecir
- [ ] use_sim_time=true tum node'larda

**Calistirma sirasi:**
```
Terminal 1: ros2 launch robot_arm bot.launch.py          # Gazebo + controllers
Terminal 2: ros2 launch robot_arm_moveit_config moveit.launch.py  # MoveIt + arm_commander + gripper
Terminal 3: ros2 launch orchestrator harvest_pipeline.launch.py   # Pipeline + orchestrator
```

---

## TASK 3: Scan -> Detection Entegrasyonu (main.py)
**Amac:** _phase_scanning placeholder'ini gercek explorer + detection pipeline ile degistir

**Yapilacak:**
- [ ] main.py `_phase_scanning()`:
  - HOME'a git
  - /explorer/panoramic_scan cagir (basitlestirilmis 3 pozisyon)
  - Explorer her pozisyonda /detection/run_at_position cagirir (zaten yapiyor)
  - Scan bittikten sonra detection pipeline'dan cluster pozisyonlarini al
  - tracked_clusters -> cluster_plans'a donustur
- [ ] spatial_detection_pipeline'dan sonuclari almak icin:
  - /detection/validate cagir (ground truth match)
  - Ya da /detection/print_results + topic subscribe ile pozisyonlari al
  - NOT: Simdilik validate sonucu yerine config pozisyonlari kullanmak da kabul edilebilir (demo icin)

---

## TASK 4: Cluster View'de Boll Detection (main.py)
**Amac:** _phase_approaching'de pre-grasp'e gidince bireysel boll 3D pozisyonlarini al

**Yapilacak:**
- [ ] main.py `_phase_approaching()`:
  - Pre-grasp cluster view'e git (arm_commander /go_to_named -> cluster_N)
  - /yolo/detect cagir (raw boll detection, /detect_clusters degil!)
  - Her boll bbox icin /depth_processor/pixel_to_3d cagir -> 3D pozisyon
  - plan.bolls listesini doldur (gercek detect edilmis pozisyonlarla)
- [ ] Fallback: YOLO detect basarisiz olursa config pozisyonunu kullan (demo guvenilirlik)
- [ ] NOT: camera_focus iteration'lari opsiyonel (sim'de yavas, demo icin skip edilebilir)

---

## TASK 5: Pick Cycle Implementasyonu
**Amac:** harvest_executor ile mock pick-and-place (gercek grip yok ama hareket tam)

**Yapilacak:**
- [ ] harvest_executor.py test:
  - Pre-grasp -> open -> boll pozisyonuna git -> close -> lift -> reservoir -> open -> pre-grasp
  - Sim'de boll fiziksel olarak tutulmayacak (sag finger calismiyor + boll buyuk)
  - Ama arm hareketi + gripper acma/kapama tam calisacak
- [ ] Reservoir pozisyonu dogrula (config: [0.0, 0.6, 0.3])
- [ ] Lift height dogrula (0.15m yeterli mi, collision var mi?)
- [ ] Test: tek boll icin /harvest/pick_boll service call

---

## TASK 6: Full Loop Kapatma
**Amac:** Orchestrator start_harvest -> scan -> approach -> detect -> pick -> next -> home

**Yapilacak:**
- [ ] main.py'de tum phase'leri bagla:
  1. /orchestrator/start_harvest trigger
  2. SCANNING: explorer panoramic scan + detection
  3. Her cluster icin:
     a. APPROACHING: pre-grasp cluster view'e git
     b. Boll detection (YOLO + depth)
     c. HARVESTING: her boll icin harvest_executor.pick_boll
     d. Pre-grasp'e don, sonraki boll
  4. RETURNING: HOME'a don
- [ ] /orchestrator/status topic'te state degisimlerini publish et
- [ ] Test: `ros2 service call /orchestrator/start_harvest std_srvs/srv/Trigger "{}"`

---

## TASK 7: Demo Gorselleri + Video
**Amac:** Midterm sunumunda gosterilecek materyaller

**Yapilacak:**
- [ ] Her asamanin screenshot'lari (YOLO annotated images zaten yolo_output/ klasorune kaydediliyor):
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

## Oncelik Sirasi

```
TASK 1 (scan basitlestir) ─┐
TASK 2 (launch file)       ├─> paralel yapilabilir
                           │
TASK 3 (scan entegrasyon)  ┘
         |
TASK 4 (boll detection at cluster view)
         |
TASK 5 (pick cycle test)
         |
TASK 6 (full loop)
         |
TASK 7 + 8 (demo + sunum)
```

---

## Notlar

### Scan Stratejisi
HOME'dan j1 rotate ile 3 pozisyon scan (robot yerinden kipirdamadan).
Ilerde (gercek robot): visual servoing ile dynamic pre-grasp view, surekli tahmin alarak yaklasma.

### Pick Cycle (ideal, ilerde)
1. Vision ML ile cluster center'lar tespit
2. HOME'den cluster'a yaklas, yaklasirken prediction al
3. En cok boll gordugu en yakin noktada dur = pre-grasp view
4. Her boll'un 3D pozisyonunu al
5. Boll topla -> pre-grasp'e don -> tekrarla -> cluster bitti
6. Home'a git, sonraki cluster

### Pick Cycle (sim demo, simdilik)
1. Config'den cluster pozisyonlari (veya scan ile detect)
2. Hardcoded pre-grasp offset (15cm)
3. YOLO + depth ile boll 3D pozisyonu (veya fallback: config)
4. Mock grip (arm hareket eder, gripper acar/kapar, ama boll fiziksel tutulmaz)
5. Sonuc: tam hareket sekansini gosteriyoruz

### Sim'de Grip Feedback
Rigid sphere — parmaklar fiziksel duruyor, pozisyon farki ile tespit. Gercekte Hand-E motor akimi (gOBJ=2, Modbus).

### MT Sunum Demo Fikri
YOLO modelin basarisini gostermek icin cesitli pozisyonlarda screenshot alip modele ver, sonuclari flow gorsellerine koy:
1. Cluster view'e yaklasirken
2. Grasp view'de durma karari
3. Identification
4. Boll approach ve pick cycle
5. Cluster bitti -> son check (bos cluster gorseli)
