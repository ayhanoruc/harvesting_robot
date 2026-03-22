# RoboCot — Doosan M1013 + Hand-E Integration Plan

**Date:** 2026-03-22
**Status:** Active

---

## Contents

- [Büyük Resim](#büyük-resim-ne-yapıyoruz)
- [Faz 0: Ortamı Hazırla](#faz-0-ortamı-hazırla-önkoşullar)
- [Faz 1: Simülasyonda M1013 + Hand-E](#faz-1-simülasyonda-m1013--hand-e-çalıştır)
- [Faz 2: Orchestrator Pipeline Adaptasyonu](#faz-2-orchestrator-pipelineı-adapte-et)
- [Faz 3: Pick-and-Place Rutini](#faz-3-pick-and-place-rutini-i̇mplemente-et)
- [Faz 4: Gerçek Hardware Deployment](#faz-4-gerçek-hardware-deployment)
- [Faz 5: Optimizasyon ve Raporlama](#faz-5-optimizasyon-ve-raporlama)
- [Zaman Çizelgesi](#zaman-çizelgesi-özeti)
- [Sim vs Real: Neler Değişiyor?](#sim-vs-real-neler-değişiyor)
- [Further Notes: Docker on Xavier](#further-notes)

---

## Büyük Resim: Ne Yapıyoruz?

Bir masa/platform üzerinde sabit duran Doosan M1013 robot kolu var. Ucunda Robotiq Hand-E tutucu ve bir RGB-D kamera var. Önünde mock cotton field var (pamuk bitkileri). Robot:

1. Kafasını çevirip tarlayı tarıyor (panoramic scan)
2. YOLO ile pamuk kozalarını tespit ediyor
3. Depth kamera ile 3D konumlarını hesaplıyor
4. Kozaları cluster'lara grupluyor
5. Her cluster'a yaklaşıp tek tek kozaları koparıyor
6. Yanındaki kutuya bırakıyor

Şu an bu pipeline'ın 1-4 arası simülasyonda çalışıyor (4-DOF arm ile). 5-6 hiç implemente edilmedi. Şimdi gerçek Doosan M1013 + Hand-E ile tamamını hem simülasyonda hem gerçekte çalıştıracağız.

---

## Faz 0: Ortamı Hazırla (Önkoşullar)

### 0.1 Development Makineni Hazırla

Senin laptop/PC'de şunlar lazım:

- Ubuntu 22.04 (ROS2 Humble için zorunlu)
- ROS2 Humble kurulu (`ros-humble-desktop`)
- Gazebo Ignition Fortress kurulu
- MoveIt2 kurulu (`ros-humble-moveit`)
- `colcon` build tool
- Mevcut `harvesting_robot` workspace'imiz çalışır durumda

Bu zaten var sende, geçen dönemden.

### 0.2 ROBOCOB Repo'larından Asset'leri Topla

Kendi repo'muza kopyalamamız gereken dosyalar:

- `robocob_ws_sim/doosan-robot/dsr_description/meshes/m1013_blue/` — 10+ DAE mesh dosyası
- `robocob_ws_sim/doosan-robot/dsr_description/meshes/m1013_collision/` — collision mesh'ler
- `robocob_ws_sim/robotiq/robotiq_description/meshes/hande/` — gripper mesh'ler (`hande.dae`, `finger_1.dae`, `finger_2.dae` + collision versiyonları)
- `robocob_ws_sim/robot_description/xacro/m1013_arm.urdf.xacro` — referans URDF (joint limits, inertial values, kinematic chain)
- `robocob_ws_sim/robotiq/robotiq_description/urdf/robotiq_hande_gripper.urdf.xacro` — gripper URDF referansı
- `robocob_ws_sim/robocob_moveit/config/` — MoveIt YAML'lar (joint_limits, kinematics, OMPL config) referans olarak

> Bunlar ROS versiyonundan bağımsız — mesh'ler ve kinematik parametreler evrensel.

## Faz 1: Simülasyonda M1013 + Hand-E Çalıştır

### Adım 1.1: Yeni URDF Oluştur

Mevcut `robot_arm/urdf/mybot.urdf.xacro` yerine yeni bir xacro yaz. Referansın ROBOCOB'un `m1013_arm.urdf.xacro`'su ama hedefin **Gazebo Ignition** formatında olması:

- 6 revolute joint (joint1-6) — limit ve inertial değerlerini ROBOCOB'dan al
- Mesh path'leri kendi repo'na göre ayarla (`package://robot_arm/meshes/m1013/`)
- `<gazebo>` tag'leri **Ignition formatında** yaz (Classic'ten farklı)
- `<ros2_control>` tag'i ekle (ROBOCOB'da `transmission` var, sen `ros2_control` hardware interface kullanacaksın — mevcut 4-DOF arm'daki pattern'in aynısı)
- `gz_ros2_control` plugin'i ekle (mevcut `mybot.urdf.xacro`'daki gibi)
- tool0 link'ine **RGB-D kamera** ekle (mevcut `camera_link`/`camera_optical_frame` yapını koru)
- tool0'a **Hand-E gripper** ekle (ROBOCOB'un xacro'sundan prismatic finger joint'leri al)

**Sonuç:** `robot_arm/urdf/m1013_robocot.urdf.xacro` — M1013 + Hand-E + Camera, Gazebo Ignition uyumlu

### Adım 1.2: Controller Config Güncelle

`robot_arm/yaml/controllers.yaml`'ı güncelle:

- `arm_controller` joint listesi: `[joint1, joint2, joint3, joint4, joint5, joint6]` (eskiden `hip, shoulder, elbow, wrist` idi)
- `gripper_controller` ekle: `[robotiq_hande_left_finger_joint]` (prismatic, 0-0.025m)
- `joint_state_broadcaster` tüm joint'leri yayınlasın

### Adım 1.3: Launch Dosyası Güncelle

`robot_arm/launch/bot.launch.py`'yi güncelle:

- Yeni URDF'i yükle
- Gazebo'da M1013'ü spawn et
- `cotton_field.world`'ü kullan
- Controller'ları sırayla başlat (`broadcaster` → `arm_controller` → `gripper_controller`)

### Adım 1.4: MoveIt2 Config Oluştur

`robot_arm_moveit_config/` paketini güncelle:

- MoveIt Setup Assistant çalıştır, yeni URDF'i yükle
- Planning group `"arm"`: joint1-6, chain `support_cube` → `tool0`
- Planning group `"gripper"`: `hande_left_finger_joint` + `hande_right_finger_joint`
- Self-collision matrix oluştur
- OMPL planner config'i kopyala (ROBOCOB'dan referans)
- Kinematics: `KDLKinematics` (ROBOCOB'da da bu)

### Adım 1.5: Test Et

```bash
# Terminal 1: Gazebo + robot spawn
ros2 launch robot_arm bot.launch.py

# Terminal 2: MoveIt
ros2 launch robot_arm_moveit_config moveit.launch.py

# Terminal 3: RViz'de Motion Planning panelinden arm'ı hareket ettir
# Gripper'ı aç/kapat test et
```

> **Milestone:** M1013 Gazebo'da görünüyor, MoveIt ile plan yapıp hareket ettirebildiğin, gripper açılıp kapandığı an.

## Faz 2: Orchestrator Pipeline'ı Adapte Et

### Adım 2.1: Joint Name Mapping

Orchestrator'daki tüm node'larda joint isimlerini güncelle:

| Eski (4-DOF) | Yeni (M1013) | Rol |
|---|---|---|
| `hip` | `joint1` | Base rotation |
| `shoulder` | `joint2` | Shoulder |
| `elbow` | `joint3` | Elbow |
| `wrist` | `joint4` | Wrist pitch |
| *(yok)* | `joint5` | Wrist roll |
| *(yok)* | `joint6` | Flange rotation |

Etkilenen dosyalar:

- `orchestrator/explorer.py` — panoramic scan joint açıları
- `orchestrator/camera_focus.py` — IBVS gain'leri ve joint mapping
- `orchestrator/spatial_detection_pipeline.py` — TF frame isimleri

### Adım 2.2: Panoramic Scan Yeniden Tasarla

Eski 4-DOF scan: hip rotation + shoulder/elbow tilt ile 7x3 grid.

M1013 ile çok daha esnek — 6-DOF sayesinde kamerayı istediğin yöne çevirebilirsin. Ama aynı mantıkla:

- `joint1` = base rotation (pan) — aynı hip gibi
- `joint2` + `joint3` = shoulder + elbow (tilt) — benzer
- `joint5` = wrist roll — kamera yönelimi için ekstra kontrol

Scan açılarını M1013'ün kinematik yapısına göre yeniden hesapla. 1300mm reach sayesinde kamera çok daha uzaktan bakabilir, FOV örtüşmesi değişir.

### Adım 2.3: Camera Focus Gain'lerini Retune Et

`camera_focus.py`'daki gain'ler (`K_hip=0.002`, `K_shoulder=0.0015`, `K_elbow=0.001`) 4-DOF arm'ın kinematiğine göre tune edilmişti. M1013'ün farklı link uzunlukları (0.62m upper arm vs 0.4m bizimki) ve farklı joint konfigürasyonu yüzünden gain'ler değişecek. Simülasyonda trial-and-error ile tune et.

### Adım 2.4: TF Frame İsimlerini Güncelle

Depth processor'da `camera_optical_frame → world` TF chain'i şimdi:

```
camera_optical_frame → camera_link → tool0 → link6 → link5 → ... → link1 → base_0 → support_cube → world
```

`depth_processor.py`'daki frame isimlerini güncelle.

### Adım 2.5: Test Et

```bash
# Tam pipeline test:
# 1. Gazebo + robot
# 2. MoveIt
# 3. YOLO detector
# 4. Depth processor
# 5. Spatial detection pipeline
# 6. Explorer (panoramic scan trigger)
# Scan başlat, 3 cluster'ı doğru tespit ettiğini doğrula
```

> **Milestone:** Panoramic scan çalışıyor, YOLO tespit ediyor, 3D pozisyonlar ~1-2cm accuracy ile hesaplanıyor — eski 4-DOF'daki sonuçların aynısı ama M1013 ile.

## Faz 3: Pick-and-Place Rutini İmplemente Et

> Bu kısım **yeni** — geçen dönem yapılmamıştı.

### Adım 3.1: Gripper Control Node Yaz

Yeni bir ROS2 node: `orchestrator/gripper_controller.py`

- **Service:** `/gripper/open` — Hand-E'yi tam aç (position=0)
- **Service:** `/gripper/close` — Hand-E'yi kapat (position=255, force=configurable)
- **Service:** `/gripper/set_position` — belirli pozisyona git
- **Topic:** `/gripper/status` — gOBJ (nesne algılama), pozisyon, akım yayınla

Simülasyonda `gripper_controller` üzerinden çalışır. Gerçekte Robotiq ROS2 driver üzerinden.

### Adım 3.2: Approach Trajectory

Cluster pozisyonu biliniyor (`spatial_detection_pipeline`'dan). Şimdi:

1. **Pre-grasp position:** Cluster'ın 15-20cm önünde, gripper boll'a bakacak şekilde orient et. M1013'ün 6-DOF'u sayesinde gripper'ı istediğin açıdan yaklaştırabilirsin (Braccio'da 4-DOF yüzünden bu kısıtlıydı).
2. **MoveIt ile plan:** `move_group` action'ı ile pre-grasp → grasp position trajectory'si oluştur
3. **Collision avoidance:** Plant gövdesini collision object olarak ekle (`landmark_publisher`'daki gibi)

### Adım 3.3: Grasp Sequence

```
1.  Move to PRE_GRASP (cluster'ın 15cm önü)
2.  Open gripper
3.  Move to GRASP (boll pozisyonu)
4.  Close gripper (force=100-150, cotton için orta seviye)
5.  Check gOBJ status:
      - gOBJ=2 (closing detected object): Başarılı, devam
      - gOBJ=3 (at position, no object): Başarısız, retry veya next boll
6.  Move to LIFT (yukarı çek)
7.  Move to RESERVOIR (kutunun üstü)
8.  Open gripper (bırak)
9.  Move to HOME
10. Next boll
```

### Adım 3.4: Orchestrator State Machine Güncelle

Mevcut state machine:

```
IDLE → DETECTING_CLUSTERS → CLUSTER_VIEW_POSITION → DETECTING_BOLLS → HARVESTING → TRANSFERRING → CLUSTER_COMPLETE
```

`HARVESTING` ve `TRANSFERRING` state'lerini implemente et (şu an sadece tanımlı, kod yok).

### Adım 3.5: Test Et

Gazebo'da cotton cluster'a yaklaş, gripper ile kavra, kutuya bırak. Tam döngü.

> **Milestone:** Simülasyonda tam harvesting cycle çalışıyor — scan → detect → approach → pick → place → next.

## Faz 4: Gerçek Hardware Deployment

### Adım 4.1: Fiziksel Kurulum

1. Doosan M1013'ü masaya/platforma monte et (controller bağlantısı + güç)
2. Controller'ı aç, teach pendant'tan servo ON yap
3. Ethernet kablosu: controller (`192.168.3.5`) → senin bilgisayar (`192.168.3.x` statik IP ver)
4. Hand-E'yi tool0 flanşına monte et
5. Hand-E güç kablosu + RS485-USB converter → senin bilgisayar USB
6. RGB-D kamerayı tool0'a monte et (custom bracket lazım, 3D print veya alüminyum L bracket)
7. Kamera USB → senin bilgisayar
8. Mock field'ı robotun önüne kur (pamuk dalları + strafor taban)
9. Reservoir bin'i robotun yanına koy

### Adım 4.2: Doosan ROS2 Driver Kur

```bash
# doosan-robot2 repo'sunu clone'la (ROS2 versiyonu)
cd ~/harvesting_ws/src
git clone https://github.com/doosan-robotics/doosan-robot2.git
cd ..
colcon build

# Test: real mode bağlantı
ros2 launch dsr_bringup2 dsr_bringup2_default.launch.py mode:=real host:=192.168.3.5 port:=12345 model:=m1013
```

Bağlandıktan sonra teach pendant'ta **"Transfer Control"** onayı ver (ROS kontrolüne geçiş).

### Adım 4.3: Hand-E ROS2 Driver Kur

```bash
# Robotiq ROS2 driver (community veya custom)
# RS485 port: /dev/ttyUSB0 veya udev rule ile /dev/robotiq_gripper
```

### Adım 4.4: Kamera Kalibrasyonu

- Gerçek kameranın K matrix'ini al (RealSense SDK veya `camera_info` topic'inden)
- `depth_processor.py`'daki `fx, fy, cx, cy` değerlerini güncelle
- P matrix bug'ından ders aldık — K matrix'i direkt kullan

### Adım 4.5: Launch Dosyasını Real Mode İçin Hazırla

Simülasyon launch'undan farklı olarak:

- Gazebo yok
- `robot_state_publisher` URDF yayınlıyor (aynı)
- Doosan ROS2 driver `mode:=real` (sim yerine)
- Gerçek kamera driver'ı (`realsense2_camera` veya `zed_ros2`)
- Gerçek gripper driver'ı
- `arm_controller` Doosan'ın controller'ını kullanıyor
- MoveIt2 aynı config ile çalışıyor

### Adım 4.6: Kalibrasyon ve Fine-Tuning

- **Camera-hand eye calibration:** Kamera tool0'a göre nerede? URDF'deki offset'i doğrula
- **Depth accuracy test:** Bilinen pozisyona koy, ölç, karşılaştır
- **Scan angle tuning:** Gerçek ortamda hangi joint açıları tüm field'ı kapsıyor?
- **Gripper force tuning:** Gerçek pamuk ile 0-255 arasında optimal force bul (muhtemelen 50-100)
- **Speed/acceleration scaling:** MoveIt2'de `velocity_scaling`, `acceleration_scaling` ayarla (güvenlik)

### Adım 4.7: Tam Sistem Testi

```bash
# Terminal 1: Doosan driver + robot_state_publisher
ros2 launch robot_arm real_bot.launch.py

# Terminal 2: MoveIt2
ros2 launch robot_arm_moveit_config moveit.launch.py

# Terminal 3: Vision pipeline
ros2 launch orchestrator detection.launch.py

# Terminal 4: Ana orchestrator
ros2 launch orchestrator harvest.launch.py
```

Butona bas, robot taramaya başlasın, pamukları tespit etsin, tek tek toplasın.

> **Milestone:** Gerçek robotta end-to-end çalışan cotton harvesting demo.

## Faz 5: Optimizasyon ve Raporlama

- YOLO model iyileştirme (gerçek ortam fotoğrafları ile retrain)
- Cycle time ölçümü (QL-04: <=60s hedefi)
- Pick success rate (QL-05: >=90% hedefi)
- Repeatability testi (QL-02: artık Doosan'ın 0.05mm'si ile trivial)
- Monitoring tool / operator dashboard
- ME492 final rapor ve demo video

---

## Zaman Çizelgesi Özeti

| Faz | Süre | Çıktı |
|---|---|---|
| Faz 0: Hazırlık | 1-2 gün | Asset'ler kopyalandı, ortam hazır |
| Faz 1: Sim'de M1013 | 5-7 gün | Gazebo'da M1013 + Hand-E + Camera çalışıyor |
| Faz 2: Pipeline adapt | 3-5 gün | YOLO + 3D detection M1013 ile çalışıyor |
| Faz 3: Pick-and-place | 5-7 gün | Tam harvesting cycle sim'de çalışıyor |
| Faz 4: Hardware deploy | 5-7 gün | Gerçek robotta çalışıyor |
| Faz 5: Optimize + rapor | ongoing | Demo hazır |

**Toplam:** ~4-6 hafta yoğun çalışma ile end-to-end demo.

---

## Sim vs Real: Neler Değişiyor?

Bu adımda **sıfır logic değişikliği** var. Aynı orchestrator kodun, aynı YOLO detector'ün, aynı pipeline'ın çalışıyor. Hiçbir Python dosyasına dokunmuyorsun.

Değişen tek şey **veri nereden geliyor ve komut nereye gidiyor:**

| Topic/Interface | Simülasyon | Gerçek | Orchestrator'ın gördüğü |
|---|---|---|---|
| `/camera/color/image_raw` | Gazebo sensor plugin üretiyor | RealSense driver üretiyor | Aynı `sensor_msgs/Image` |
| `/camera/depth/image_raw` | Gazebo sensor plugin üretiyor | RealSense driver üretiyor | Aynı `sensor_msgs/Image` |
| `/camera/depth/camera_info` | Gazebo üretiyor | RealSense driver üretiyor | Aynı `sensor_msgs/CameraInfo` |
| `/joint_states` | `gz_ros2_control` yayınlıyor | Doosan ROS2 driver yayınlıyor | Aynı `sensor_msgs/JointState` |
| `/tf` | Gazebo + `robot_state_publisher` | Doosan driver + `robot_state_publisher` | Aynı TF tree |
| MoveIt trajectory execution | Gazebo controller'a gidiyor | Doosan controller'a gidiyor | Aynı `MoveGroup` action |
| Gripper command | Gazebo joint controller | Robotiq ROS2 driver | Aynı interface |

Orchestrator `/camera/color/image_raw` topic'ini dinliyor — o topic'e kimin yazdığını bilmiyor, umursamıyor. Gazebo da yazsa, RealSense da yazsa, aynı `sensor_msgs/Image` geliyor.

> Bu tam olarak **ROS'un varlık sebebi** — hardware abstraction. Node'lar birbirleriyle topic/service/action üzerinden konuşuyor, altındaki hardware'i bilmiyorlar.

Tek yapman gereken **launch dosyasında** Gazebo satırlarını gerçek driver satırlarıyla değiştirmek. Bir nevi config swap — `sim.launch.py` vs `real.launch.py`.

**Tek dikkat edilecek şey:** Kamera intrinsikleri değişecek (Gazebo'da `fx=fy=277`, gerçek kamerada farklı olacak). Ama bu da `camera_info` topic'inden otomatik okunuyor — `depth_processor.py` zaten K matrix'i `camera_info` callback'inden alıyor, hardcoded değil. O da otomatik adapte olacak.



---

## Further Notes

### Docker on Xavier — How It Works

```
┌─────────────────────────────────────────────┐
│ Jetson AGX Xavier                            │
│ Host: Ubuntu 20.04 + JetPack 5.x            │
│                                              │
│  ┌─────────────────────────────────────┐     │
│  │ Docker Container                    │     │
│  │ Ubuntu 22.04                        │     │
│  │ ROS2 Humble                         │     │
│  │ MoveIt2, YOLO11, our orchestrator   │     │
│  │ CUDA 11.4 (passthrough from host)   │     │
│  │ 512 CUDA cores + 64GB unified mem   │     │
│  └─────────────────────────────────────┘     │
│                                              │
│  ROBOCOB's ROS1 stack can run separately     │
│  on host or in its own container             │
└─────────────────────────────────────────────┘
```

NVIDIA's `jetson-containers` project (`github.com/dusty-nv/jetson-containers`) has pre-built images like `dustynv/ros:humble-desktop-l4t-r35.x.x` that are tested on Xavier with JetPack 5.

The `--runtime nvidia` flag gives the container direct access to the GPU. YOLO11s inference on Xavier's 512 Volta CUDA cores would run comfortably.

### Deployment Options

**Option A: Your PC runs everything (Path C as planned)**

- Simplest, no Docker complexity
- Laptop connects to Doosan via Ethernet, Hand-E via USB, camera via USB
- YOLO runs on laptop GPU or CPU
- Xavier untouched

**Option B: Xavier runs our stack via Docker** *(recommended for final demo)*

- More powerful GPU for inference (Volta 512 cores + 64GB unified memory)
- No need to carry a laptop to the lab
- Xavier handles compute, talks to Doosan controller on same subnet (`192.168.3.x` — already on the same network!)
- Camera can stay USB-connected to Xavier (it already has USB ports)
- Hand-E RS485 also connects to Xavier's USB

> Option B is cleaner for the final demo — everything runs on one box that's already physically co-located with the robot and on the same network.


### Physical Architecture (Option B)

```
                    ┌──────────────────────────────────────────┐
                    │           ROBOCOB SYSTEM (existing)      │
                    │                                          │
  ┌─────────┐      │  ┌────────────────────────────────────┐  │
  │ Doosan  │      │  │  Jetson AGX Xavier (192.168.3.4)   │  │
  │ M1013   │      │  │  Ubuntu 20.04 + JetPack 5.x       │  │
  │Controller├──────┤  │                                    │  │
  │192.168. │ ETH  │  │  ┌──────────────────────────────┐  │  │
  │ 3.5:    │(already) │  │  Docker Container             │  │  │
  │ 12345   │on same│  │  Ubuntu 22.04 + ROS2 Humble   │  │  │
  └────┬────┘subnet │  │  MoveIt2 + YOLO11 + Our code  │  │  │
       │         │  │  │  doosan-robot2 ROS2 driver     │  │  │
  ┌────┴────┐    │  │  │  Robotiq Hand-E ROS2 driver    │  │  │
  │ Teach   │    │  │  │  Camera driver (realsense2)    │  │  │
  │ Pendant │    │  │  │  512 CUDA cores for YOLO       │  │  │
  └─────────┘    │  │  └──────────────────────────────┘  │  │
                 │  │                                    │  │
                 │  │  Host still runs ROBOCOB's ROS1    │  │
                 │  │  (mobile base, MQTT, lidar, etc)   │  │
                 │  │  -- we don't touch any of that     │  │
                 │  └────────────────────────────────────┘  │
                 │            │USB           │USB            │
                 │     ┌──────┴──┐    ┌──────┴──────┐       │
                 │     │Hand-E   │    │ RGB-D Camera│       │
                 │     │RS485-USB│    │ (RealSense  │       │
                 │     │Converter│    │  or other)  │       │
                 │     └─────────┘    └─────────────┘       │
                 │                                          │
  ┌──────────┐   │     ┌────────────┐                       │
  │ Hokuyo   ├───┤     │ ACU Card   │                       │
  │ Lidar    │   │     │ 192.168.   │                       │
  │192.168.  │   │     │ 3.3        │                       │
  │ 3.7      │   │     └────────────┘                       │
  └──────────┘   │     (mobile base -- we ignore these)     │
                 └──────────────────────────────────────────┘

  ┌──────────────┐
  │ Your Laptop  │  SSH / VNC into Xavier for monitoring
  │ (any OS)     │  No ROS2 needed on laptop for deployment
  └──────────────┘
```

### Network Map (192.168.3.x subnet — all pre-wired)

| Device | IP | Port | Protocol | We Use? |
|---|---|---|---|---|
| Xavier (robot PC) | `192.168.3.4` | — | SSH (password: `1`) | **YES** — our compute |
| Doosan M1013 controller | `192.168.3.5` | `12345` | TCP (DRFL API) | **YES** — arm control |
| ACU card (mobile base) | `192.168.3.3` | `1883` (MQTT) | MQTT | NO |
| Hokuyo lidar | `192.168.3.7` | — | Ethernet | NO |
| Hand-E gripper | `/dev/robotiq_gripper` | — | RS485 Modbus (USB) | **YES** — gripper control |
| RGB-D camera | USB | — | USB3 | **YES** — vision |

> **Key insight:** Xavier, Doosan controller, and all peripherals are already on the same physical network and wired up. We don't need to run any new cables for the arm+gripper. The only physical addition is mounting a camera on tool0.


### Step-by-Step Deployment

#### STEP 0: Lab Visit Recon

Do these checks physically at the robot:

- [ ] SSH into Xavier: `ssh robocob@192.168.3.4` (password: `1`)
- [ ] Check JetPack version: `cat /etc/nv_tegra_release` or `dpkg -l | grep nvidia-jetpack`
- [ ] Check Docker installed: `docker --version` and `docker run --runtime nvidia --rm nvidia/cuda:11.4-base nvidia-smi`
- [ ] Check disk space: `df -h` (Docker images need ~10-15GB)
- [ ] Ping Doosan controller: `ping 192.168.3.5`
- [ ] Check USB ports: `lsusb` — identify Hand-E RS485 converter (vendor `0403:6001` or `0403:6015` or `10c4:ea60`)
- [ ] Check if Hand-E is connected: `ls /dev/ttyUSB*`
- [ ] Check available USB ports for camera mounting
- [ ] Photo the tool0 flange — measure bolt pattern for camera bracket
- [ ] Check if Doosan controller is powered on, check teach pendant status
- [ ] Check what processes Xavier is running: `sudo service robocob status`, `rostopic list`

#### STEP 1: Docker Setup on Xavier

```bash
# SSH into Xavier
ssh robocob@192.168.3.4

# Verify NVIDIA runtime works
sudo docker run --rm --runtime nvidia nvidia/cuda:11.4.0-base-ubuntu20.04 nvidia-smi

# Pull or build ROS2 Humble container for JetPack 5
# Option A: Use dusty-nv's pre-built (recommended)
sudo docker pull dustynv/ros:humble-desktop-l4t-r35.4.1

# Option B: Build custom Dockerfile (if pre-built doesn't match JetPack version)
# We'd write a Dockerfile based on nvcr.io/nvidia/l4t-base:r35.x.x
```

#### STEP 2: Create Our Docker Image (extends base with our deps)

```dockerfile
FROM dustynv/ros:humble-desktop-l4t-r35.4.1

# Install our dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# YOLO + Python deps
RUN pip3 install ultralytics opencv-python-headless numpy PyYAML pymodbus

# Robotiq Hand-E: pymodbus is the only dep (Modbus RTU over serial)

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copy our code
COPY src/ /ros2_ws/src/

# Build
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Source on entry
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
```

#### STEP 3: Run Container with Hardware Access

```bash
sudo docker run -it --rm \
    --runtime nvidia \
    --network host \
    --privileged \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=0 \
    --name robocot \
    robocot:latest \
    bash
```

Flags explained:

- `--network host` — container uses Xavier's network directly, can reach Doosan at `192.168.3.5`
- `--privileged` + `-v /dev:/dev` — access USB devices (Hand-E RS485, camera)
- `--runtime nvidia` — GPU access for YOLO inference

#### STEP 4: Inside Container — Launch Real Robot

```bash
# Terminal 1: Doosan ROS2 driver (connects to real controller)
ros2 launch dsr_bringup2 dsr_bringup2_default.launch.py \
    mode:=real host:=192.168.3.5 port:=12345 model:=m1013

# Terminal 2: robot_state_publisher + MoveIt2
ros2 launch robot_arm real_bot.launch.py
ros2 launch robot_arm_moveit_config moveit.launch.py

# Terminal 3: Camera driver
ros2 launch realsense2_camera rs_launch.py \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30

# Terminal 4: Gripper driver (our custom Modbus RTU node)
ros2 run orchestrator gripper_controller \
    --ros-args -p port:=/dev/robotiq_gripper \
    -p baudrate:=115200 -p slave_id:=9

# Terminal 5: Vision + orchestrator pipeline
ros2 launch orchestrator harvest.launch.py
```

#### STEP 5: Physical Setup for Camera

The M1013's tool0 flange has a standard **ISO 9409-1-50-4-M6** mounting pattern (50mm bolt circle, 4x M6 bolts). Camera mounting options:

- 3D print an L-bracket that bolts to the flange
- Aluminum L-bracket from hardware store
- Camera goes on the side of the gripper (not blocking the grasp axis)

Hand-E is already mounted on tool0 via the `gripper_root_link` (the xacro shows `gripper_root_joint` fixed to `link6`).

Camera mounts **beside** the Hand-E, attached to `link6`/`tool0`. URDF offset needs to match physical mounting.

### What We DON'T Need to Touch

| ROBOCOB Component | Why We Skip It |
|---|---|
| MQTT broker + `robot_communication` | Mobile base only |
| ACU card (`192.168.3.3`) | Motor/encoder for wheels |
| Hokuyo lidar (`192.168.3.7`) | Navigation, not picking |
| `move_base` / AMCL / gmapping | We're fixed-base |
| `error_manager` topics | Mobile base monitoring |
| Heartbeat system | Mobile base safety |
| STO (Safe Torque Off) for mobile | Mobile base |

> The Doosan controller has its own independent safety (E-stop on teach pendant, joint limits in firmware). We interact with it purely over TCP `192.168.3.5:12345` via the `doosan-robot2` ROS2 driver.



### Development Workflow

```
┌─────────────────┐        git push         ┌──────────────┐
│  Senin PC (WSL) │ ──────────────────────► │   GitHub     │
│  develop + sim  │                         └──────┬───────┘
└────────┬────────┘                                │
         │                                    git pull
         │ ssh robocob@192.168.3.4                 │
         │                                  ┌──────▼───────┐
         └─────────────────────────────────►│   Xavier      │
           (lab'da veya uzaktan VPN ile)    │  docker run   │
                                            │  gerçek robot │
                                            └───────────────┘
```

> Senin PC'de asla Docker kurmana gerek yok. Docker sadece Xavier'da çalışıyor.

**İlk sefer (lab'da SSH ile):**

```bash
# 1. Xavier'a bağlan
ssh robocob@192.168.3.4

# 2. Repo'yu clone'la
git clone https://github.com/azurmalachite/harvesting_ws.git
cd harvesting_ws

# 3. Docker image'ı build et (~15-20 dk)
sudo docker build -t robocot .

# 4. Container'ı çalıştır
sudo docker run -it --runtime nvidia --network host --privileged \
    -v /dev:/dev -v $(pwd):/ros2_ws robocot bash

# 5. İçeride: colcon build && ros2 launch ...
```

**Sonraki günlerde kod güncellemek için:**

```bash
ssh robocob@192.168.3.4
cd harvesting_ws
git pull
# Container zaten volume mount ile bağlı, ya rebuild ya da içeride colcon build
```

> Lab'daki tek fiziksel iş: ilk gün ethernet/USB kabloları ve kamerayı kontrol etmek, teach pendant'tan onay vermek. Geri kalan her şey remote.



### Xavier Detailed Architecture

```
┌─── Jetson AGX Xavier (192.168.3.4) ──────────────────────────────────┐
│                                                                       │
│  ┌─── HOST OS (Ubuntu 20.04) ──────────────────────────────────────┐ │
│  │                                                                  │ │
│  │  ROBOCOB'un kendi ROS1 stack'i (biz dokunmuyoruz)               │ │
│  │  - roscore                                                       │ │
│  │  - robot_communication (MQTT ↔ ACU, mobil base)                 │ │
│  │  - error_manager                                                 │ │
│  │  - navigation (AMCL, move_base)                                  │ │
│  │  → Bunlar çalışmaya devam edebilir, bize engel değil            │ │
│  │                                                                  │ │
│  │  Docker Engine + NVIDIA Runtime                                  │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│                                                                       │
│  ┌─── DOCKER CONTAINER (Ubuntu 22.04 + ROS2 Humble) ──────────────┐ │
│  │                                                                  │ │
│  │  doosan-robot2 (ROS2 driver) ─── TCP ──► 192.168.3.5:12345     │ │
│  │  MoveIt2 (path planner) ─── trajectory ──► doosan driver        │ │
│  │  robot_state_publisher ─── /joint_states ──► /tf                │ │
│  │  gripper_node (pymodbus) ─── Modbus RTU ──► /dev/ttyUSB0       │ │
│  │  real_yolo_detector (YOLO11 + CUDA) ─── GPU inference           │ │
│  │  realsense2_camera ─── /dev/video*                              │ │
│  │  orchestrator (explorer, spatial_pipeline, camera_focus, etc.)   │ │
│  └──────────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────────┘
```

### Servo Nasıl Tetikleniyor: Sinyal Akışı

```
Kullanıcı: "Pamukları topla"
         │
         ▼
   ┌─────────────┐
   │ Orchestrator │  "cluster_1'e git"
   └──────┬──────┘
          ▼
   ┌─────────────┐
   │   MoveIt2   │  IK çözer, trajectory hesaplar
   │             │  [joint1=0.5, joint2=-1.2, joint3=0.8, ...]
   └──────┬──────┘
          ▼
   ┌──────────────────┐
   │  doosan-robot2   │  ROS2 paketi, container İÇİNDE çalışıyor
   │  (ROS2 driver)   │  trajectory'yi TCP paketlerine çeviriyor
   └──────┬───────────┘
          │ TCP soketi (--network host sayesinde)
          ▼
   ┌──────────────────┐
   │ Doosan Controller│  AYRI FİZİKSEL KUTU (192.168.3.5:12345)
   │ (AC/DC box)      │  Kendi firmware'i var
   │                  │  Servo sürücüler burada
   │                  │  PID kontrol burada
   │                  │  Güvenlik limitleri burada
   └──────┬───────────┘
          │ Güç kablosu (robot cable)
          ▼
   ┌──────────────────┐
   │ M1013 Robot Kolu  │  6 eklemdeki servo motorlar
   │ (fiziksel)        │  joint1-6 hareket eder
   └──────────────────┘
```

> **Yani:** Bizim kodumuz servo'ya doğrudan voltaj göndermiyor. `doosan-robot2` driver'ı sadece "joint1'i 0.5 radyana götür" diyor TCP üzerinden. Asıl motor kontrolü (PID, akım limiti, güvenlik) Doosan'ın kendi controller kutusunda oluyor. Bu yüzden güvenli — biz saçma bir komut göndersek bile controller'ın firmware'i reddediyor.

### Gripper Aynı Mantık

```
gripper_node (container içinde)
        │ pymodbus kütüphanesi
        │ Modbus RTU komutları
        ▼
USB port (/dev/ttyUSB0)
        │ fiziksel kablo
        ▼
RS485-USB converter
        │
        ▼
Hand-E'nin kendi mikrodenetleyicisi
        │ motor sürüyor
        ▼
Parmaklar açılır/kapanır
```

### Docker İçi vs Dışı Özet

| Docker İÇİNDE | Docker DIŞINDA (Xavier host) | Docker DIŞINDA (fiziksel) |
|---|---|---|
| ROS2 Humble | ROBOCOB ROS1 stack (dokunmuyoruz) | Doosan controller kutusu |
| `doosan-robot2` (ROS2 driver) | Docker engine | M1013 robot kolu |
| MoveIt2 | NVIDIA CUDA runtime | Hand-E gripper |
| YOLO11 inference | Linux kernel + USB drivers | RS485-USB converter |
| Orchestrator pipeline | Ethernet stack | RGB-D kamera |
| Gripper node (pymodbus) | | Teach pendant |
| Camera driver (realsense2) | | |
| `robot_state_publisher` | | |

- `--network host` → container'a Xavier'ın ethernet'ini veriyor → Doosan'a ulaşır
- `--privileged -v /dev:/dev` → container'a USB portları veriyor → Kamera ve Hand-E'ye ulaşır
- `--runtime nvidia` → container'a GPU veriyor → YOLO çalışır