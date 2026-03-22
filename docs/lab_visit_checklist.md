# Lab Visit Checklist — Day 3 (2026-03-24)

## Yanında Getir
- [ ] Laptop + şarj kablosu
- [ ] Ethernet kablosu (yedek)
- [ ] USB-C hub (Xavier'a bağlanmak için gerekebilir)
- [ ] Telefon (fotoğraf/video için)

---

## 1. Xavier'a SSH Bağlantı

```bash
# Xavier IP: 192.168.3.4, password: 1 (INTEGRATION.md'den)
ssh robocob@192.168.3.4
```

- [ ] SSH bağlantısı başarılı
- [ ] Şifre `1` doğru mu? Değilse not al: _____

---

## 2. Xavier Sistem Durumu

```bash
# JetPack versiyonu (Docker base image seçimi için kritik)
cat /etc/nv_tegra_release
dpkg -l | grep nvidia-jetpack

# Ubuntu versiyonu (20.04 bekleniyor)
lsb_release -a

# Disk alanı (Docker image ~15GB lazım)
df -h

# GPU test
sudo tegrastats | head -5

# Mevcut ROS1 stack çalışıyor mu?
sudo service robocob status
rostopic list 2>/dev/null | head -10

# Çalışan Docker container var mı?
sudo docker ps -a
```

| Bilgi | Değer |
|---|---|
| JetPack versiyonu | _____ (5.x bekleniyor) |
| Ubuntu versiyonu | _____ (20.04 bekleniyor) |
| Disk boş alan | _____ GB (min 15GB lazım) |
| ROBOCOB ROS1 çalışıyor mu | Evet / Hayır |

---

## 3. Docker + NVIDIA Runtime

```bash
# Docker kurulu mu?
docker --version

# NVIDIA runtime var mı?
sudo docker info 2>/dev/null | grep -i runtime

# GPU test (JetPack 5.x = CUDA 11.4)
sudo docker run --rm --runtime nvidia nvidia/cuda:11.4.0-base-ubuntu20.04 nvidia-smi

# İnternet var mı? (Docker pull için)
ping -c 3 8.8.8.8
curl -s https://hub.docker.com > /dev/null && echo "Docker Hub OK" || echo "Docker Hub FAIL"

# ROS2 Humble pre-built container test
# JetPack versiyonuna göre tag'i ayarla (r35.1.0, r35.2.1, r35.3.1, r35.4.1)
sudo docker pull dustynv/ros:humble-desktop-l4t-r35.4.1
sudo docker run --rm --runtime nvidia dustynv/ros:humble-desktop-l4t-r35.4.1 ros2 --help
```

- [ ] Docker kurulu
- [ ] NVIDIA runtime var
- [ ] GPU container'dan erişilebilir
- [ ] İnternet erişimi var
- [ ] ROS2 Humble container pull edilebiliyor
- [ ] JetPack versiyonuna uygun container tag'i: _____

---

## 4. Network Ping Testleri

```
Lab Network (192.168.3.x subnet — hepsi önceden kablolu):
  Xavier:           192.168.3.4
  ACU control card: 192.168.3.3
  Doosan M1013:     192.168.3.5 (port 12345)
  Hokuyo lidar:     192.168.3.7
```

### Xavier üzerinden
```bash
ping -c 3 192.168.3.5    # Doosan controller (ZORUNLU)
ping -c 3 192.168.3.3    # ACU (opsiyonel, biz kullanmıyoruz)
ping -c 3 192.168.3.7    # Lidar (opsiyonel)

# Doosan controller port testi (12345 açık mı?)
nc -zv 192.168.3.5 12345
```

### Laptop'tan (lab WiFi veya ethernet ile bağlanınca)
```bash
ping -c 3 192.168.3.4    # Xavier
ping -c 3 192.168.3.5    # Doosan controller
```

- [ ] Xavier → Doosan ping OK
- [ ] Doosan port 12345 açık (nc test)
- [ ] Laptop → Xavier ping OK
- [ ] Laptop IP: _____
- [ ] Subnet mask: _____

---

## 5. Doosan M1013 Controller

```bash
# Teach pendant'ı kontrol et:
# 1. Robot power ON
# 2. Servo ON
# 3. Manuel modda jointleri hareket ettir
# 4. Home pozisyona götür
```

- [ ] Controller kutusu power ON
- [ ] Teach pendant açılıyor
- [ ] Servo ON yapılabiliyor
- [ ] Manuel modda kol hareket ediyor
- [ ] Emergency stop lokasyonu not alındı
- [ ] Teach pendant'ta "Transfer Control" (ROS'a kontrol devri) nasıl yapılıyor?

---

## 6. Hand-E Gripper (RS485)

```bash
# Xavier üzerinde USB cihazları listele
lsusb
# RS485 converter aranıyor: vendor 0403:6001, 0403:6015, veya 10c4:ea60

# Serial port kontrol
ls /dev/ttyUSB*
ls /dev/ttyACM*

# Udev rule var mı? (ROBOCOB bunu yapmış olabilir)
ls /dev/robotiq_gripper 2>/dev/null && echo "Udev rule var" || echo "Udev rule yok"

# Basit serial bağlantı testi
python3 -c "
import serial
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
print('Port açıldı:', s.name)
s.close()
print('OK')
" 2>&1
```

- [ ] RS485 USB converter bağlı
- [ ] lsusb'de converter görünüyor (vendor: _____)
- [ ] Port adı: /dev/ttyUSB___ veya /dev/robotiq_gripper
- [ ] Gripper fiziksel olarak parmakları hareket ettiriyor (elle test veya teach pendant'tan)
- [ ] Serial port açılabiliyor

---

## 7. Kamera

```bash
# USB kameralar
ls /dev/video*
v4l2-ctl --list-devices 2>/dev/null

# RealSense bağlıysa (ROBOCOB'da D435 var)
rs-enumerate-devices 2>/dev/null
realsense-viewer 2>/dev/null  # GUI varsa

# Kamera tipi ve bağlantı
lsusb | grep -iE '(intel|realsense|camera)'
```

- [ ] Kamera tipi: _____ (RealSense D435 bekleniyor)
- [ ] Kamera Xavier'a USB ile bağlı mı?
- [ ] Görüntü alınabiliyor
- [ ] Kamera tool0'a nasıl monte edilmiş? (fotoğraf çek)
- [ ] Ek kamera montaj aparatı lazım mı?

---

## 8. Tool0 Flange Ölçümü

M1013'ün tool0 flanşı: ISO 9409-1-50-4-M6 (50mm bolt circle, 4x M6)

- [ ] Flange fotoğrafı çekildi
- [ ] Hand-E zaten monte mi?
- [ ] Kamera için yer var mı? (L-bracket lazım mı?)
- [ ] Kablo routing notu: gripper + kamera kabloları nereden geçiyor?

---

## 9. Fotoğraf/Video Çek (Sunum İçin)

- [ ] ROBOCOB genel görünüm (robot + platform + çevre)
- [ ] M1013 kol yakın çekim (joint'ler, kablolar)
- [ ] Hand-E gripper yakın çekim (açık + kapalı)
- [ ] Tool0 flange + kamera montaj noktası
- [ ] Xavier kutusu ve kablolama
- [ ] Teach pendant ekranı
- [ ] Network switch / ethernet bağlantıları
- [ ] Controller kutusu (AC/DC box)
- [ ] Lab ortamı genel (sunumda context için)
- [ ] 10-15sn video: teach pendant'tan kolu hareket ettir

---

## 10. Docker Ön Hazırlık (Zaman Varsa)

```bash
# Repo'yu Xavier'a clone'la
cd ~
git clone https://github.com/ayhanoruc/harvesting_robot.git
cd harvesting_ws

# Docker image build (internet varsa, ~15-20 dk)
sudo docker build -t robocot .

# Container'ı test amaçlı çalıştır
sudo docker run -it --rm \
    --runtime nvidia \
    --network host \
    --privileged \
    -v /dev:/dev \
    -v $(pwd):/ros2_ws \
    --name robocot \
    robocot:latest \
    bash

# İçeride:
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# Doosan driver bağlantı testi (robot aktifse)
ros2 launch dsr_bringup2 dsr_bringup2_default.launch.py \
    mode:=real host:=192.168.3.5 port:=12345 model:=m1013
```

- [ ] Repo clone'landı
- [ ] Docker image build başarılı
- [ ] Container içinden `ros2 --help` çalışıyor
- [ ] Doosan driver bağlantı testi yapıldı

---

## 11. Toplanan Bilgiler Özeti

Lab'dan dönerken bunlar dolu olsun:

| Bilgi | Değer |
|---|---|
| Xavier JetPack versiyonu | |
| Xavier Ubuntu | |
| Xavier disk boş alan | |
| Docker + NVIDIA runtime | |
| Container tag (JetPack uyumlu) | |
| Doosan IP:port ping/nc | |
| Doosan teach pendant şifresi | |
| Hand-E RS485 port adı | |
| Kamera tipi + bağlantı | |
| Xavier SSH şifresi | |
| Laptop IP (lab subnet) | |
| İnternet erişimi | |
| E-stop lokasyonu | |
| Kamera montaj durumu | |

---

## Sonraki Adım (Lab'dan Sonra)

Lab bilgilerine göre:
1. Dockerfile'ı Xavier JetPack versiyonuna göre finalize et (container tag)
2. `real_bot.launch.py` yaz — Doosan ROS2 driver + kamera + gripper
3. Doosan driver config'ini IP/port'a göre ayarla
4. Hand-E Modbus config'ini port adına göre ayarla
5. Kamera intrinsiklerini `camera_info`'dan doğrula
6. Camera mounting bracket tasarla (gerekiyorsa)
