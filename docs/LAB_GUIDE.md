# Lab Deployment Guide — Xavier'a Docker Kurulumu

**Tarih:** 2026-04-17
**Hedef:** Xavier'a Docker image'i kurup temel testler yapılacak

---

## Genel Bakis

Xavier'da Docker container icinde ROS2 Humble stack'imizi calistiriyoruz. Container Xavier'in GPU'suna, network'une ve USB cihazlarina erisebilmeli.

```
Xavier (Ubuntu 20.04 host)
  |
  +-- Docker Container (Ubuntu 22.04 + ROS2 Humble)
       |-- --runtime nvidia   -> GPU (CUDA, YOLO inference)
       |-- --network host     -> Doosan controller 192.168.3.5:12345
       |-- --privileged       -> USB cihazlari (kamera, gripper)
       |-- -v /dev:/dev       -> /dev/ttyUSB*, /dev/video*
```

---

## Baglanti Bilgileri

| Cihaz | IP / Adres | Kullanici | Sifre |
|-------|-----------|-----------|-------|
| Xavier | 192.168.3.4 | robocob | 1 |
| Doosan controller | 192.168.3.5:12345 | - | - |
| Hand-E gripper | /dev/ttyUSB0 (RS485) | - | - |
| Kamera | /dev/video* (USB) | - | - |

---

## Adim 1: Xavier'a SSH Baglan

```bash
ssh robocob@192.168.3.4
# Sifre: 1
```

---

## Adim 2: Sistem Kontrolu

```bash
# JetPack versiyonu (Docker base image secimi icin KRITIK)
cat /etc/nv_tegra_release
# Ornek cikti: "# R35 (release), REVISION: 4.1, ..."
# Bu "r35.4.1" demek

# Disk alani (en az 15GB bos lazim)
df -h /

# Docker kurulu mu?
docker --version

# NVIDIA runtime var mi?
sudo docker info 2>/dev/null | grep -i runtime

# Internet var mi? (Docker pull icin gerekli)
ping -c 3 8.8.8.8

# Doosan controller erisilebilir mi?
ping -c 3 192.168.3.5
nc -zv 192.168.3.5 12345

# USB cihazlari
lsusb
ls /dev/ttyUSB* /dev/video*
```

### Sonuclari Not Alin

| Bilgi | Deger |
|-------|-------|
| L4T versiyonu (orn: r35.4.1) | _____ |
| Bos disk alani | _____ GB |
| Docker kurulu mu | Evet / Hayir |
| NVIDIA runtime var mi | Evet / Hayir |
| Internet erisimi | Evet / Hayir |
| Doosan ping OK mi | Evet / Hayir |
| Port 12345 acik mi | Evet / Hayir |
| USB serial portlar | _____ |
| Video cihazlari | _____ |

---

## Adim 3: Kodu Xavier'a Aktar (scp)

Kod Xavier'da degil, kendi makinenizde. scp ile gonderin:

```bash
# KENDI MAKINENIZDE (Windows terminal / git bash):
scp -r harvesting_robot robocob@192.168.3.4:~/harvesting_ws/src
# sifre: 1
```

Sonra Xavier'da kontrol edin:
```bash
# Xavier SSH icinde:
ls ~/harvesting_ws/src/Dockerfile
ls ~/harvesting_ws/src/orchestrator
# Ikisi de gorunuyorsa OK

# veya deploy script ile:
cd ~/harvesting_ws
bash src/scripts/xavier_deploy.sh verify
```

---

## Adim 4: Docker Image Build

```bash
cd ~/harvesting_ws

# ONEMLI: L4T_TAG'i Adim 2'deki versiyona gore ayarla!
# Ornek: JetPack 5.1.2 -> r35.4.1
#         JetPack 5.1.1 -> r35.3.1
#         JetPack 5.1.0 -> r35.2.1

sudo docker build \
    --build-arg L4T_TAG=r35.4.1 \
    -t robocot \
    -f src/Dockerfile \
    src/

# Bu 15-30 dakika surebilir (internet hizina bagli)
```

### Build Basarisiz Olursa

**"dustynv/ros:humble-desktop-l4t-rXX.X.X not found" hatasi:**
L4T_TAG yanlis. `cat /etc/nv_tegra_release` ciktisindaki versiyonu kullanin.

**Disk alani yetersiz:**
```bash
# Eski Docker imajlarini temizle
sudo docker system prune -a
```

**Internet yok:**
Image'i onceden baska makinede build edip tar olarak tasiyabilirsiniz:
```bash
# Build eden makinede:
sudo docker save robocot | gzip > robocot.tar.gz
# Xavier'a kopyala:
scp robocot.tar.gz robocob@192.168.3.4:~/
# Xavier'da yukle:
sudo docker load < robocot.tar.gz
```

---

## Adim 5: Container'i Calistir

```bash
sudo docker run -it \
    --runtime nvidia \
    --network host \
    --privileged \
    -v /dev:/dev \
    -v ~/harvesting_ws:/ros2_ws_host \
    -e ROS_DOMAIN_ID=0 \
    --name robocot \
    robocot:latest \
    bash
```

### Container Icinde Birden Fazla Terminal Acma

Container `docker run` ile baslatildiktan sonra tek bir bash shell'desiniz.
Ek terminal acmak icin **Xavier'da yeni bir SSH oturumu** acip ayni container'a baglanin:

```bash
# Xavier'a yeni SSH (laptop'tan):
ssh robocob@192.168.3.4

# Calisan container'a gir:
sudo docker exec -it robocot bash

# Icinde source yap:
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

Istediginiz kadar terminal acabilirsiniz — hepsi ayni container, ayni dosya sistemi, ayni ROS2 ortami:
```
SSH terminal 1 -> docker run ... (container'i baslatti, icinde)
SSH terminal 2 -> docker exec -it robocot bash (ayni container, 2. shell)
SSH terminal 3 -> docker exec -it robocot bash (ayni container, 3. shell)
```

### Container Durduktan Sonra Tekrar Baslatma

```bash
# Container durmussa (exit yaptiysaniz):
sudo docker start -i robocot

# Container'i silip sifirdan baslatmak icin:
sudo docker rm robocot
# ve yukaridaki docker run komutunu tekrar calistirin
```

### Flag Aciklamalari (HOCAYA ACIKLAMA ICIN)

| Flag | Ne Yapar | Neden Lazim |
|------|----------|-------------|
| `--runtime nvidia` | Container'a Xavier'in GPU'sunu verir | YOLO inference CUDA ile calismali |
| `--network host` | Container, Xavier'in network stack'ini kullanir | 192.168.3.5:12345'e TCP ile erismek icin |
| `--privileged` | Container'a tum /dev erisimi verir | USB cihazlar: RS485, kamera |
| `-v /dev:/dev` | Host /dev'i container'a mount eder | /dev/ttyUSB0 (gripper), /dev/video* (kamera) |
| `-v ~/harvesting_ws:/ros2_ws_host` | Kodu host'tan paylaşır | Container disinda duzenleme icin |
| `-e ROS_DOMAIN_ID=0` | ROS2 domain ayari | Host'taki ROS1 ile carpismasin |

### Guvenlik Notlari

- `--privileged` tum cihazlara erisim verir. Uretim ortaminda bunun yerine sadece gerekli cihazlari mount etmek daha guvenli:
  ```bash
  --device=/dev/ttyUSB0 --device=/dev/video0
  ```
- `--network host` container'i izole etmez. Uretimde bridge network + port expose daha guvenli.
- Biz lab/demo ortamindayiz, bu seviye yeterli.

---

## Adim 6: Container Icinde Testler

Container bash'i actiktan sonra:

### Test 1: ROS2 Calisiyor mu?
```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 pkg list | grep -E "orchestrator|robot_arm|harvester"
```
Beklenen: `harvester_interfaces`, `orchestrator`, `robot_arm`, `robot_arm_moveit_config`

### Test 2: GPU Erisimi
```bash
python3 -c "
import torch
print('CUDA available:', torch.cuda.is_available())
if torch.cuda.is_available():
    print('Device:', torch.cuda.get_device_name(0))
    print('Memory:', round(torch.cuda.get_device_properties(0).total_mem / 1e9, 1), 'GB')
"
```
Beklenen: `CUDA available: True`, Xavier'in GPU'su gorunmeli

### Test 3: YOLO Model Yuklenebiliyor mu?
```bash
python3 -c "
from ultralytics import YOLO
import os
# Find model path
model_path = '/ros2_ws/install/orchestrator/share/orchestrator/models/best.pt'
if not os.path.exists(model_path):
    model_path = '/ros2_ws/src/orchestrator/models/best.pt'
m = YOLO(model_path)
print('Model loaded OK')
print('Classes:', m.names)
"
```
Beklenen: `Model loaded OK`, `{0: 'cotton_boll', 1: 'cotton_boll-cluster', 2: 'unripe-cotton'}`

### Test 4: Doosan Network Erisimi (Container icinden)
```bash
ping -c 3 192.168.3.5
nc -zv 192.168.3.5 12345
```
Beklenen: `--network host` sayesinde ayni sonuc

### Test 5: USB Cihazlar (Container icinden)
```bash
ls /dev/ttyUSB*
ls /dev/video*
```
Beklenen: Host'taki ayni cihazlar gorunmeli

### Test 6: Custom Interface'ler
```bash
ros2 interface show harvester_interfaces/srv/YoloDetect
ros2 interface show harvester_interfaces/srv/HarvestBoll
ros2 interface show harvester_interfaces/msg/BoundingBox
```

---

## Adim 7: Doosan ROS2 Driver Testi (Opsiyonel)

Eger Doosan driver container icinde build edildiyse VE robot aciksa:

```bash
# Container icinde, ayri terminalde
ros2 launch dsr_bringup2 dsr_bringup2_default.launch.py \
    mode:=real host:=192.168.3.5 port:=12345 model:=m1013
```

**DIKKAT:**
- Teach pendant'ta **remote mode** aktif olmali
- Robot **servo ON** durumunda olmali
- E-stop konumunu bilin
- Ilk defa calistirirken yaninda biri olsun

---

## Sorun Giderme

### "Permission denied" USB erisiminde
```bash
# Container disinda (Xavier host'ta)
sudo chmod 666 /dev/ttyUSB0
# veya container'i --privileged ile calistirdiginizdan emin olun
```

### "OCI runtime create failed: nvidia" hatasi
```bash
# NVIDIA container toolkit kontrolu
sudo apt-get install nvidia-container-toolkit
sudo systemctl restart docker
```

### Container'a yeniden baglanma
```bash
# Container acikken baska terminal:
sudo docker exec -it robocot bash

# Container durmussa:
sudo docker start -i robocot
```

### Kodu guncelleme (container durdurmadan)
```bash
# Xavier host'ta:
cd ~/harvesting_ws/src && git pull

# Container icinde:
cd /ros2_ws_host
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Ozet: Minimum Basari Kriterleri

Labdan donmeden once bunlar tamamlanmis olmali:

- [ ] Xavier SSH OK
- [ ] JetPack versiyonu not alindi: _____
- [ ] Docker image build OK
- [ ] Container icinde `ros2 pkg list` calisyor
- [ ] Container icinde GPU erisilebilir (CUDA OK)
- [ ] Container icinden `ping 192.168.3.5` OK
- [ ] Container icinden USB portlar gorunuyor
- [ ] YOLO model yuklenebiliyor

### Bonus (vakit kalirsa)

Bunlarin hepsi **container icinden** yapiliyor.

#### B1. Doosan ROS2 Driver Baglanti Testi

Once Doosan driver'i build etmeyi deneyin:
```bash
# Container icinde:
cd /ros2_ws/src
git clone https://github.com/doosan-robotics/doosan-robot2.git
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dsr_msgs2 dsr_bringup2 2>&1 | tail -10
# Hata alirsa not alin, hangi dependency eksik yazan
```

Build basarili olduysa VE robot aciksa:
```bash
# 1. Teach pendant'ta remote mode ACIK olmali
# 2. Robot servo ON olmali
# 3. E-stop konumunu bilin!

# Driver'i baslat:
ros2 launch dsr_bringup2 dsr_bringup2_default.launch.py \
    mode:=real host:=192.168.3.5 port:=12345 model:=m1013

# Basarili olursa /joint_states topic'i akmaya baslar:
# (ayri terminalde: sudo docker exec -it robocot bash)
ros2 topic echo /joint_states --once
```
- [ ] Doosan driver build oldu mu: Evet / Hayir (hata: _____)
- [ ] Driver robota baglandi mi: Evet / Hayir
- [ ] `/joint_states` topic'i geliyor mu: Evet / Hayir

#### B2. Hand-E Serial Port Testi

```bash
# Container icinde:

# 1. Hangi port?
ls /dev/ttyUSB* /dev/robotiq_gripper 2>/dev/null
# Not alin: _____

# 2. Port acilabiliyor mu?
python3 -c "
import serial
port = '/dev/ttyUSB0'  # veya hangi port bulduysaniz
try:
    s = serial.Serial(port, 115200, timeout=1)
    print('Port acildi:', s.name)
    s.close()
    print('OK')
except Exception as e:
    print('HATA:', e)
"

# 3. Modbus ile gripper'a ulasilabiliyor mu? (opsiyonel)
python3 -c "
from pymodbus.client import ModbusSerialClient
client = ModbusSerialClient(port='/dev/ttyUSB0', baudrate=115200, timeout=1)
if client.connect():
    result = client.read_holding_registers(0x07D0, 3, slave=9)
    if not result.isError():
        print('Gripper Modbus OK! Registers:', result.registers)
    else:
        print('Modbus read error:', result)
    client.close()
else:
    print('Modbus connect FAIL')
"
```
- [ ] Serial port bulundu: _____ (orn: /dev/ttyUSB0)
- [ ] Port acilabiliyor: Evet / Hayir
- [ ] Modbus baglanti: Evet / Hayir

#### B3. Kamera Testi

```bash
# Container icinde:

# 1. Video cihazlari
ls /dev/video*
v4l2-ctl --list-devices 2>/dev/null

# 2. RealSense ise:
# (realsense2-camera container'da kurulu degilse skip edin)
rs-enumerate-devices 2>/dev/null | head -20

# 3. Basit frame capture testi
python3 -c "
import cv2
cap = cv2.VideoCapture(0)  # veya 2, 4 deneyin
if cap.isOpened():
    ret, frame = cap.read()
    if ret:
        print('Frame OK:', frame.shape)
        cv2.imwrite('/ros2_ws_host/test_frame.jpg', frame)
        print('Kaydedildi: ~/harvesting_ws/test_frame.jpg')
    else:
        print('Frame alinamadi')
    cap.release()
else:
    print('Kamera acilamadi')
"
```
- [ ] Video device bulundu: _____
- [ ] Kamera tipi: _____ (RealSense D435 / USB cam / diger)
- [ ] Frame alindi: Evet / Hayir

#### B4. Fotograflar (Telefonla)

- [ ] ROBOCOB genel gorunum (robot + platform)
- [ ] M1013 kol yakin cekim (joint'ler, kablolar)
- [ ] Hand-E gripper yakin cekim
- [ ] Tool0 flange + kamera montaj noktasi
- [ ] Xavier kutusu ve kablolama
- [ ] Teach pendant ekrani
- [ ] Controller kutusu (AC/DC box)
- [ ] E-stop konumu
- [ ] Kablo routing (Hand-E kablosu nereye gidiyor? Direkt USB mi, Doosan controller uzerinden mi?)
