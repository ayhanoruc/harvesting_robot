Büyük Resim: Ne Yapıyoruz?
Bir masa/platform üzerinde sabit duran Doosan M1013 robot kolu var. Ucunda Robotiq Hand-E tutucu ve bir RGB-D kamera var. Önünde mock cotton field var (pamuk bitkileri). Robot:

Kafasını çevirip tarlayı tarıyor (panoramic scan)
YOLO ile pamuk kozalarını tespit ediyor
Depth kamera ile 3D konumlarını hesaplıyor
Kozaları cluster'lara grupluyor
Her cluster'a yaklaşıp tek tek kozaları koparıyor
Yanındaki kutuya bırakıyor
Şu an bu pipeline'ın 1-4 arası simülasyonda çalışıyor (4-DOF arm ile). 5-6 hiç implemente edilmedi. Şimdi gerçek Doosan M1013 + Hand-E ile tamamını hem simülasyonda hem gerçekte çalıştıracağız.

Faz 0: Ortamı Hazırla (Önkoşullar)
0.1 Development Makineni Hazırla
Senin laptop/PC'de şunlar lazım:

Ubuntu 22.04 (ROS2 Humble için zorunlu)
ROS2 Humble kurulu (ros-humble-desktop)
Gazebo Ignition Fortress kurulu
MoveIt2 kurulu (ros-humble-moveit)
colcon build tool
Mevcut harvesting_robot workspace'imiz çalışır durumda
Bu zaten var sende, geçen dönemden.

0.2 ROBOCOB Repo'larından Asset'leri Topla
Kendi repo'muza kopyalamamız gereken dosyalar:

robocob_ws_sim/doosan-robot/dsr_description/meshes/m1013_blue/ -- 10+ DAE mesh dosyası
robocob_ws_sim/doosan-robot/dsr_description/meshes/m1013_collision/ -- collision mesh'ler
robocob_ws_sim/robotiq/robotiq_description/meshes/hande/ -- gripper mesh'ler (hande.dae, finger_1.dae, finger_2.dae + collision versiyonları)
robocob_ws_sim/robot_description/xacro/m1013_arm.urdf.xacro -- referans URDF (joint limits, inertial values, kinematic chain)
robocob_ws_sim/robotiq/robotiq_description/urdf/robotiq_hande_gripper.urdf.xacro -- gripper URDF referansı
robocob_ws_sim/robocob_moveit/config/ -- MoveIt YAML'lar (joint_limits, kinematics, OMPL config) referans olarak
Bunlar ROS versiyonundan bağımsız -- mesh'ler ve kinematik parametreler evrensel.

Faz 1: Simülasyonda M1013 + Hand-E Çalıştır
Adım 1.1: Yeni URDF Oluştur
Mevcut robot_arm/urdf/mybot.urdf.xacro yerine yeni bir xacro yaz. Referansın ROBOCOB'un m1013_arm.urdf.xacro'su ama hedefin Gazebo Ignition formatında olması:

6 revolute joint (joint1-6) -- limit ve inertial değerlerini ROBOCOB'dan al
Mesh path'leri kendi repo'na göre ayarla (package://robot_arm/meshes/m1013/)
<gazebo> tag'leri Ignition formatında yaz (Classic'ten farklı)
<ros2_control> tag'i ekle (ROBOCOB'da transmission var, sen ros2_control hardware interface kullanacaksın -- mevcut 4-DOF arm'daki pattern'in aynısı)
gz_ros2_control plugin'i ekle (mevcut mybot.urdf.xacro'daki gibi)
tool0 link'ine RGB-D kamera ekle (mevcut camera_link/camera_optical_frame yapını koru)
tool0'a Hand-E gripper ekle (ROBOCOB'un xacro'sundan prismatic finger joint'leri al)
Sonuç: robot_arm/urdf/m1013_robocot.urdf.xacro -- M1013 + Hand-E + Camera, Gazebo Ignition uyumlu

Adım 1.2: Controller Config Güncelle
robot_arm/yaml/controllers.yaml'ı güncelle:

arm_controller joint listesi: [joint1, joint2, joint3, joint4, joint5, joint6] (eskiden hip, shoulder, elbow, wrist idi)
gripper_controller ekle: [robotiq_hande_left_finger_joint] (prismatic, 0-0.025m)
joint_state_broadcaster tüm joint'leri yayınlasın
Adım 1.3: Launch Dosyası Güncelle
robot_arm/launch/bot.launch.py'yi güncelle:

Yeni URDF'i yükle
Gazebo'da M1013'ü spawn et
cotton_field.world'ü kullan
Controller'ları sırayla başlat (broadcaster -> arm_controller -> gripper_controller)
Adım 1.4: MoveIt2 Config Oluştur
robot_arm_moveit_config/ paketini güncelle:

MoveIt Setup Assistant çalıştır, yeni URDF'i yükle
Planning group "arm": joint1-6, chain support_cube -> tool0
Planning group "gripper": hande_left_finger_joint + hande_right_finger_joint
Self-collision matrix oluştur
OMPL planner config'i kopyala (ROBOCOB'dan referans)
Kinematics: KDLKinematics (ROBOCOB'da da bu)
Adım 1.5: Test Et

# Terminal 1: Gazebo + robot spawn
ros2 launch robot_arm bot.launch.py

# Terminal 2: MoveIt
ros2 launch robot_arm_moveit_config moveit.launch.py

# Terminal 3: RViz'de Motion Planning panelinden arm'ı hareket ettir
# Gripper'ı aç/kapat test et
Milestone: M1013 Gazebo'da görünüyor, MoveIt ile plan yapıp hareket ettirebildiğin, gripper açılıp kapandığı an.

Faz 2: Orchestrator Pipeline'ı Adapte Et
Adım 2.1: Joint Name Mapping
Orchestrator'daki tüm node'larda joint isimlerini güncelle:

Eski (4-DOF)	Yeni (M1013)	Rol
hip	joint1	Base rotation
shoulder	joint2	Shoulder
elbow	joint3	Elbow
wrist	joint4	Wrist pitch
(yok)	joint5	Wrist roll
(yok)	joint6	Flange rotation
Etkilenen dosyalar:

orchestrator/explorer.py -- panoramic scan joint açıları
orchestrator/camera_focus.py -- IBVS gain'leri ve joint mapping
orchestrator/spatial_detection_pipeline.py -- TF frame isimleri
Adım 2.2: Panoramic Scan Yeniden Tasarla
Eski 4-DOF scan: hip rotation + shoulder/elbow tilt ile 7x3 grid.

M1013 ile çok daha esnek -- 6-DOF sayesinde kamerayı istediğin yöne çevirebilirsin. Ama aynı mantıkla:

joint1 = base rotation (pan) -- aynı hip gibi
joint2 + joint3 = shoulder + elbow (tilt) -- benzer
joint5 = wrist roll -- kamera yönelimi için ekstra kontrol
Scan açılarını M1013'ün kinematik yapısına göre yeniden hesapla. 1300mm reach sayesinde kamera çok daha uzaktan bakabilir, FOV örtüşmesi değişir.

Adım 2.3: Camera Focus Gain'lerini Retune Et
camera_focus.py'daki gain'ler (K_hip=0.002, K_shoulder=0.0015, K_elbow=0.001) 4-DOF arm'ın kinematiğine göre tune edilmişti. M1013'ün farklı link uzunlukları (0.62m upper arm vs 0.4m bizimki) ve farklı joint konfigürasyonu yüzünden gain'ler değişecek. Simülasyonda trial-and-error ile tune et.

Adım 2.4: TF Frame İsimlerini Güncelle
Depth processor'da camera_optical_frame -> world TF chain'i şimdi:


camera_optical_frame -> camera_link -> tool0 -> link6 -> link5 -> ... -> link1 -> base_0 -> support_cube -> world
depth_processor.py'daki frame isimlerini güncelle.

Adım 2.5: Test Et

# Tam pipeline test:
# 1. Gazebo + robot
# 2. MoveIt
# 3. YOLO detector
# 4. Depth processor
# 5. Spatial detection pipeline
# 6. Explorer (panoramic scan trigger)
# Scan başlat, 3 cluster'ı doğru tespit ettiğini doğrula
Milestone: Panoramic scan çalışıyor, YOLO tespit ediyor, 3D pozisyonlar ~1-2cm accuracy ile hesaplanıyor -- eski 4-DOF'daki sonuçların aynısı ama M1013 ile.

Faz 3: Pick-and-Place Rutini İmplemente Et
Bu kısım yeni -- geçen dönem yapılmamıştı.

Adım 3.1: Gripper Control Node Yaz
Yeni bir ROS2 node: orchestrator/gripper_controller.py

Service: /gripper/open -- Hand-E'yi tam aç (position=0)
Service: /gripper/close -- Hand-E'yi kapat (position=255, force=configurable)
Service: /gripper/set_position -- belirli pozisyona git
Topic: /gripper/status -- gOBJ (nesne algılama), pozisyon, akım yayınla
Simülasyonda gripper_controller üzerinden çalışır. Gerçekte Robotiq ROS2 driver üzerinden.

Adım 3.2: Approach Trajectory
Cluster pozisyonu biliniyor (spatial_detection_pipeline'dan). Şimdi:

Pre-grasp position: Cluster'ın 15-20cm önünde, gripper boll'a bakacak şekilde orient et. M1013'ün 6-DOF'u sayesinde gripper'ı istediğin açıdan yaklaştırabilirsin (Braccio'da 4-DOF yüzünden bu kısıtlıydı).
MoveIt ile plan: move_group action'ı ile pre-grasp -> grasp position trajectory'si oluştur
Collision avoidance: Plant gövdesini collision object olarak ekle (landmark_publisher'daki gibi)
Adım 3.3: Grasp Sequence

1. Move to PRE_GRASP (cluster'ın 15cm önü)
2. Open gripper
3. Move to GRASP (boll pozisyonu)
4. Close gripper (force=100-150, cotton için orta seviye)
5. Check gOBJ status:
   - gOBJ=2 (closing detected object): Başarılı, devam
   - gOBJ=3 (at position, no object): Başarısız, retry veya next boll
6. Move to LIFT (yukarı çek)
7. Move to RESERVOIR (kutunun üstü)
8. Open gripper (bırak)
9. Move to HOME
10. Next boll
Adım 3.4: Orchestrator State Machine Güncelle
Mevcut state machine:


IDLE -> DETECTING_CLUSTERS -> CLUSTER_VIEW_POSITION -> DETECTING_BOLLS -> HARVESTING -> TRANSFERRING -> CLUSTER_COMPLETE
HARVESTING ve TRANSFERRING state'lerini implemente et (şu an sadece tanımlı, kod yok).

Adım 3.5: Test Et
Gazebo'da cotton cluster'a yaklaş, gripper ile kavra, kutuya bırak. Tam döngü.

Milestone: Simülasyonda tam harvesting cycle çalışıyor -- scan -> detect -> approach -> pick -> place -> next.

Faz 4: Gerçek Hardware Deployment
Adım 4.1: Fiziksel Kurulum
Doosan M1013'ü masaya/platforma monte et (controller bağlantısı + güç)
Controller'ı aç, teach pendant'tan servo ON yap
Ethernet kablosu: controller (192.168.3.5) -> senin bilgisayar (192.168.3.x statik IP ver)
Hand-E'yi tool0 flanşına monte et
Hand-E güç kablosu + RS485-USB converter -> senin bilgisayar USB
RGB-D kamerayı tool0'a monte et (custom bracket lazım, 3D print veya alüminyum L bracket)
Kamera USB -> senin bilgisayar
Mock field'ı robotun önüne kur (pamuk dalları + strafor taban)
Reservoir bin'i robotun yanına koy
Adım 4.2: Doosan ROS2 Driver Kur

# doosan-robot2 repo'sunu clone'la (ROS2 versiyonu)
cd ~/harvesting_ws/src
git clone https://github.com/doosan-robotics/doosan-robot2.git
cd ..
colcon build

# Test: real mode bağlantı
ros2 launch dsr_bringup2 dsr_bringup2_default.launch.py mode:=real host:=192.168.3.5 port:=12345 model:=m1013
Bağlandıktan sonra teach pendant'ta "Transfer Control" onayı ver (ROS kontrolüne geçiş).

Adım 4.3: Hand-E ROS2 Driver Kur

# Robotiq ROS2 driver (community veya custom)
# RS485 port: /dev/ttyUSB0 veya udev rule ile /dev/robotiq_gripper
Adım 4.4: Kamera Kalibrasyonu
Gerçek kameranın K matrix'ini al (RealSense SDK veya camera_info topic'inden)
depth_processor.py'daki fx, fy, cx, cy değerlerini güncelle
P matrix bug'ından ders aldık -- K matrix'i direkt kullan
Adım 4.5: Launch Dosyasını Real Mode İçin Hazırla
Simülasyon launch'undan farklı olarak:

Gazebo yok
robot_state_publisher URDF yayınlıyor (aynı)
Doosan ROS2 driver mode:=real (sim yerine)
Gerçek kamera driver'ı (realsense2_camera veya zed_ros2)
Gerçek gripper driver'ı
arm_controller Doosan'ın controller'ını kullanıyor
MoveIt2 aynı config ile çalışıyor
Adım 4.6: Kalibrasyon ve Fine-Tuning
Camera-hand eye calibration: Kamera tool0'a göre nerede? URDF'deki offset'i doğrula
Depth accuracy test: Bilinen pozisyona koy, ölç, karşılaştır
Scan angle tuning: Gerçek ortamda hangi joint açıları tüm field'ı kapsıyor?
Gripper force tuning: Gerçek pamuk ile 0-255 arasında optimal force bul (muhtemelen 50-100)
Speed/acceleration scaling: MoveIt2'de velocity_scaling, acceleration_scaling ayarla (güvenlik)
Adım 4.7: Tam Sistem Testi

# Terminal 1: Doosan driver + robot_state_publisher
ros2 launch robot_arm real_bot.launch.py

# Terminal 2: MoveIt2
ros2 launch robot_arm_moveit_config moveit.launch.py

# Terminal 3: Vision pipeline
ros2 launch orchestrator detection.launch.py

# Terminal 4: Ana orchestrator
ros2 launch orchestrator harvest.launch.py
Butona bas, robot taramaya başlasın, pamukları tespit etsin, tek tek toplasın.

Milestone: Gerçek robotta end-to-end çalışan cotton harvesting demo.

Faz 5: Optimizasyon ve Raporlama
YOLO model iyileştirme (gerçek ortam fotoğrafları ile retrain)
Cycle time ölçümü (QL-04: <=60s hedefi)
Pick success rate (QL-05: >=90% hedefi)
Repeatability testi (QL-02: artık Doosan'ın 0.05mm'si ile trivial)
Monitoring tool / operator dashboard
ME492 final rapor ve demo video
Zaman Çizelgesi Özeti
Faz	Süre	Çıktı
Faz 0: Hazırlık	1-2 gün	Asset'ler kopyalandı, ortam hazır
Faz 1: Sim'de M1013	5-7 gün	Gazebo'da M1013 + Hand-E + Camera çalışıyor
Faz 2: Pipeline adapt	3-5 gün	YOLO + 3D detection M1013 ile çalışıyor
Faz 3: Pick-and-place	5-7 gün	Tam harvesting cycle sim'de çalışıyor
Faz 4: Hardware deploy	5-7 gün	Gerçek robotta çalışıyor
Faz 5: Optimize + rapor	ongoing	Demo hazır
Toplam: ~4-6 hafta yoğun çalışma ile end-to-end demo.

Nereden başlamak istiyorsun?

Adım 4.5: Launch Dosyasını Real Mode İçin Hazırla
Simülasyon launch'undan farklı olarak:

Gazebo yok
robot_state_publisher URDF yayınlıyor (aynı)
Doosan ROS2 driver mode:=real (sim yerine)
Gerçek kamera driver'ı (realsense2_camera veya zed_ros2)
Gerçek gripper driver'ı
arm_controller Doosan'ın controller'ını kullanıyor
MoveIt2 aynı config ile çalışıyor
--
bu adımda aslında kodda logic olarak bi değişiklik filan olmicak mı yani?
Evet, tam olarak öyle. Bu adımda sıfır logic değişikliği var.

Aynı orchestrator kodun, aynı YOLO detector'ün, aynı spatial_detection_pipeline'ın, aynı explorer'ın, aynı camera_focus'un çalışıyor. Hiçbir Python dosyasına dokunmuyorsun.

Değişen tek şey veri nereden geliyor ve komut nereye gidiyor:

Topic/Interface	Simülasyon	Gerçek	Orchestrator'ın gördüğü
/camera/color/image_raw	Gazebo sensor plugin üretiyor	RealSense driver üretiyor	Aynı sensor_msgs/Image
/camera/depth/image_raw	Gazebo sensor plugin üretiyor	RealSense driver üretiyor	Aynı sensor_msgs/Image
/camera/depth/camera_info	Gazebo üretiyor	RealSense driver üretiyor	Aynı sensor_msgs/CameraInfo
/joint_states	gz_ros2_control yayınlıyor	Doosan ROS2 driver yayınlıyor	Aynı sensor_msgs/JointState
/tf	Gazebo + robot_state_publisher	Doosan driver + robot_state_publisher	Aynı TF tree
MoveIt trajectory execution	Gazebo controller'a gidiyor	Doosan controller'a gidiyor	Aynı MoveGroup action
Gripper command	Gazebo joint controller	Robotiq ROS2 driver	Aynı interface
Orchestrator "/camera/color/image_raw" topic'ini dinliyor -- o topic'e kimin yazdığını bilmiyor, umursamıyor. Gazebo da yazsa, RealSense da yazsa, aynı sensor_msgs/Image geliyor.

Bu tam olarak ROS'un varlık sebebi -- hardware abstraction. Node'lar birbirleriyle topic/service/action üzerinden konuşuyor, altındaki hardware'i bilmiyorlar. Geçen dönem bu mimariyi kurarken "modüler olsun, node'lar bağımsız çalışsın" diye uğraştığımız şey tam da bu an için.

Tek yapman gereken launch dosyasında Gazebo satırlarını gerçek driver satırlarıyla değiştirmek. Bir nevi config swap -- sim.launch.py vs real.launch.py. Raporında yazdığın gibi: "When we needed to update the YOLO detector, only that node changed. Everything downstream stayed the same."

Aynı prensip. Gazebo'yu kaldır, gerçek driver'ları koy, pipeline'ın geri kalanı farkı anlamıyor bile.

Tek dikkat edilecek şey: Kamera intrinsikleri değişecek (Gazebo'da fx=fy=277, gerçek kamerada farklı olacak). Ama bu da camera_info topic'inden otomatik okunuyor -- depth_processor.py zaten K matrix'i camera_info callback'inden alıyor, hardcoded değil. O da otomatik adapte olacak.