# Raspberry Pi 4 + Camera Module V2 NoIR + Ubuntu 24.04 + ROS 2 Jazzy

Bu dosya; kameranın Ubuntu'da neden siyah görüntü verdiğini, nasıl çalışır hale getirdiğimizi ve kullandığımız her komutun ne işe yaradığını anlatır.

Tarih: 14 Kasım 2026

---

## 1. Kısa özet

| | |
|---|---|
| Donanım | Raspberry Pi 4 (8 GB), Camera Module V2 NoIR (Sony IMX219 sensör, IR filtresi yok) |
| İşletim sistemi | Ubuntu 24.04.4 LTS (noble), ROS 2 Jazzy |
| Sorun | Raspberry Pi OS'ta çalışan kamera, Ubuntu'da siyah görüntü veriyordu |
| Çözüm | Raspberry Pi'nin libcamera çatalı (v0.8.0), `camera_ros` ile birlikte `~/camera_ws` içinde kaynaktan derlendi |
| Sonuç | Görüntü ROS topic'i olarak yayınlanıyor. Test karesinin ortalama parlaklığı 125.6 (siyah olsaydı 0.0 çıkardı) |

Kamera donanımı baştan beri sağlamdı. Sorun, görüntüyü işleyen yazılım katmanındaydı.

---

## 2. Katmanlar: kameradan koda

```
Kamera (IMX219 sensör)
   |  CSI şerit kablo
   v
Çekirdek (kernel) sürücüleri
   - imx219  : sensörü yönetir (config.txt / cihaz ağacı ile tanımlanır)
   - unicam  : CSI girişinden ham veriyi alır  -> /dev/video0
   - bcm2835-isp : Pi'nin görüntü işlemcisi (ISP)
   |
   v
libcamera (kullanıcı alanı kütüphanesi)
   - ISP'yi sürer, otomatik pozlama / beyaz dengesi gibi ayarları yapar
   - "IPA" modülleri bu algoritmaları çalıştırır
   |
   v
Uygulama
   - camera_ros (ROS düğümü)  -> /camera/image_raw topic'i
   - kendi yazacağın kod (ör. QR çözümleyici) bu topic'e abone olur
```

Önemli nokta: `/dev/video0` ham sensör verisi (Bayer) verir, hazır bir fotoğraf vermez. Fotoğrafı ISP ve libcamera üretir.

---

## 3. Neden siyah ekran?

- OpenCV'de `cv2.VideoCapture(0)` doğrudan `/dev/video0`'ı okur. Bu düğüm ham Bayer verisi taşıdığı için sonuç siyah (ortalama 0.0) çıktı.
- Doğru yol, görüntüyü libcamera üzerinden almak. Raspberry Pi OS bunu hazır getirir. Ubuntu 24.04'te ise libcamera'nın Raspberry Pi'ye uygun, çalışan bir sürümü hazır gelmiyor.

---

## 4. Ne denedik, ne çıktı

| # | Denediğimiz | Sonuç | Çıkarım |
|---|---|---|---|
| 1 | `v4l2-ctl --list-devices` | `unicam` ve `bcm2835-isp` görünüyor | Çekirdek tarafı hazır |
| 2 | `sudo dmesg \| grep imx219` | `/soc/i2c0mux/i2c@1/imx219@10` bulundu | Sensör tanımlı |
| 3 | `cam -l` (Ubuntu'nun libcamera 0.2.0'ı) | Kamera listelendi | libcamera Pi hattını tanıyor |
| 4 | `cam -c 1 --capture=5 ...` | `FATAL ... prepareIsp` hatası, program çöktü | Ubuntu'nun libcamera 0.2.0'ında hata var (büyük ihtimalle bilinen bir hata) |
| 5 | OpenCV `VideoCapture(0)` | "Başarılı" dedi ama ortalama parlaklık 0.0 | Kare siyahtı. Başarı mesajı yanıltıcıydı |
| 6 | `apt install ros-jazzy-camera-ros` (ROS'un libcamera 0.7'si) | Farklı çökme: IPA işleminde hata + zaman aşımları | ROS'un hazır libcamera paketi de çalışmadı |
| 7 | Raspberry Pi libcamera çatalı + camera_ros, `colcon build` | 17 dk 32 sn'de bitti | Derleme başarılı |
| 8 | `kontrol.py` | ortalama 125.6, en parlak 255 | Görüntü geliyor |
| 9 | `scp` ile Windows'a kopyalama | Görüntü göründü (NoIR olduğu için IR tonlu) | Doğrulandı |

---

## 5. Komutlar tek tek

### 5.1 Teşhis komutları

| Komut | Ne yapar |
|---|---|
| `lsb_release -a` | Ubuntu sürümünü gösterir. Önemli, çünkü kamera yığını Ubuntu 25.04 ve sonrasında hazır geliyor, 24.04'te gelmiyor. |
| `sudo dmesg \| grep -i -E "imx219\|unicam"` | Çekirdek mesajlarında sensörü arar. `sudo` gerekir, çünkü Ubuntu normal kullanıcıya çekirdek logunu kapatır. |
| `grep -i -E "camera\|dtoverlay\|start_x\|gpu_mem" /boot/firmware/config.txt` | Pi'nin önyükleme ayarlarındaki kamera satırlarını gösterir. `camera_auto_detect=1` kamerayı otomatik algılar. `dtoverlay=imx219` sensörü elle tanımlar. |
| `v4l2-ctl --list-devices` | Video cihazlarını listeler. `unicam` = CSI kamera girişi, `bcm2835-isp` = görüntü işlemci. |
| `free -h` | RAM miktarı (derleme için yeterli mi diye baktık: 7.6 GB). |
| `printenv ROS_DISTRO` | Kurulu ROS sürümü (`jazzy`). |

### 5.2 libcamera'yı sistem paketleriyle deneme

| Komut | Ne yapar |
|---|---|
| `sudo dpkg --configure -a` | Yarım kalmış paket kurulumunu tamamlar (önceki hata: `dpkg was interrupted`). |
| `sudo apt install -y libcamera-tools libcamera-ipa` | libcamera'nın test araçlarını (`cam`, `qcam`) ve IPA modüllerini kurar. |
| `cam -l` | libcamera'nın bulduğu kameraları listeler. |
| `cam -c 1 --capture=5 --file=/tmp/kare-#.ppm` | 1 numaralı kameradan 5 kare çekip dosyaya kaydetmeyi dener. Burada çöktü. |

### 5.3 ROS'un hazır paketini deneme

| Komut | Ne yapar |
|---|---|
| `sudo apt install -y ros-jazzy-camera-ros` | ROS düğümünü ve ROS'un kendi libcamera paketini (`/opt/ros/jazzy` altına) kurar. |
| `source /opt/ros/jazzy/setup.bash` | Terminale ROS'un yollarını yükler. Her yeni terminalde gerekir. |
| `ros2 run camera_ros camera_node --ros-args -p width:=1280 -p height:=960` | Kamera düğümünü başlatır ve görüntüyü ROS topic'ine yayınlar. |

`1280x960` seçmemizin sebebi: `camera_ros` dokümanına göre V2 kamerada 640x480 istenirse libcamera çok kırpılmış bir sensör modu seçiyor (yaklaşık 2.5 kat dijital zoom). QR çözümü için geniş görüş alanı daha iyi.

### 5.4 Çalışan çözümü kurma (kaynaktan derleme)

**Paketler:**

```
sudo apt install -y python3-colcon-common-extensions python3-colcon-meson python3-rosdep \
  meson ninja-build python3-yaml python3-ply python3-jinja2
```

| Paket | Ne işe yarar |
|---|---|
| `python3-colcon-common-extensions` | `colcon`: ROS projelerini derleme aracı |
| `python3-colcon-meson` | `colcon`'a Meson ile derlenen projeleri (libcamera) öğretir |
| `meson`, `ninja-build` | libcamera'nın derleme sistemi |
| `python3-ply`, `python3-jinja2`, `python3-yaml` | libcamera'nın derleme sırasında kod üretmek için kullandığı Python kütüphaneleri |
| `python3-rosdep` | ROS bağımlılıklarını otomatik kuran araç |

(Diğer geliştirme paketleri bu Pi'de zaten kuruluydu. `libgnutls28-dev` gibi bazıları, deponun `noble-updates` kaynağı eksik olduğu için çakışma verdi. Zorunlu olmadıkları için çıkardık.)

**rosdep ayarı:**

```
sudo rosdep init || true
rosdep update
```

`rosdep init` bir kez yapılır (`|| true`: zaten yapılmışsa hata verse de devam et). `rosdep update` ROS paket listesini indirir.

**Kaynakları indirme:**

```
mkdir -p ~/camera_ws/src && cd ~/camera_ws/src
git clone https://github.com/raspberrypi/libcamera.git
git clone https://github.com/christianrauch/camera_ros.git
```

`~/camera_ws` bir "çalışma alanı": `src` içine kaynak kodlar konur, derleme çıktıları `build` ve `install` klasörlerine yazılır. Raspberry Pi'nin kendi libcamera çatalı (Raspberry Pi OS'un kullandığı sürüm) ve ROS düğümü burada.

**ROS bağımlılıklarını çözme** (bu Pi'de gerekmedi, çünkü `ros-jazzy-camera-ros` kurulumu bağımlılıkları zaten getirmişti; temiz bir Pi'de gerekir):

```
cd ~/camera_ws
source /opt/ros/jazzy/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro jazzy --skip-keys=libcamera
```

`--skip-keys=libcamera`: libcamera'yı ROS deposundan istemiyoruz, kendimiz derliyoruz.

**Derleme:**

```
colcon build --event-handlers=console_direct+ 2>&1 | tee build.log
```

Önce libcamera, sonra camera_ros derlenir. Bu Pi 4'te 17 dakika 32 saniye sürdü. `tee build.log` çıktıyı hem ekrana hem dosyaya yazar, hata olursa dosyadan bakılır. Sürüm olarak libcamera 0.8.0 derlendi. Derleme sırasında libpisp alt projesi kendi bağımlılığını GitHub'dan indirdi, bu normal.

**Kullanıma hazırlama:**

```
source ~/camera_ws/install/setup.bash
```

Bu komut, terminalin bizim derlediğimiz libcamera'yı ve camera_ros'u bulmasını sağlar. Her yeni terminalde tekrar gerekir. Yapmazsan ROS'un eski, çöken libcamera'sı devreye girer.

### 5.5 Test

| Komut | Ne yapar |
|---|---|
| `ros2 run camera_ros camera_node --ros-args -p width:=1280 -p height:=960` | Kamerayı açar, görüntüyü `/camera/image_raw` topic'ine yayınlar. Bu terminal açık kalmalı. |
| `ros2 topic hz /camera/image_raw` | Saniyede kaç kare geldiğini gösterir. |
| `python3 ~/kontrol.py` | Topic'e abone olur, 30. kareyi alır, parlaklığını yazar ve `ros_test.jpg` olarak kaydeder. |

`kontrol.py` içeriği:

```python
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2


class T(Node):
    def __init__(self):
        super().__init__('kontrol')
        self.n = 0
        self.create_subscription(Image, '/camera/image_raw', self.cb, 10)

    def cb(self, m):
        self.n += 1
        if self.n < 30:  # pozlama oturana kadar bekle
            return
        a = np.frombuffer(m.data, dtype=np.uint8)
        print('kodlama:', m.encoding, 'boyut:', m.width, 'x', m.height,
              'ortalama:', a.mean(), 'en parlak:', a.max())
        ch = m.step // m.width
        img = a.reshape(m.height, m.width, ch)
        cv2.imwrite('ros_test.jpg', img[:, :, :3])
        raise SystemExit


rclpy.init()
n = T()
try:
    rclpy.spin(n)
except SystemExit:
    pass
```

İlk 30 kare atlanır, çünkü kamera açıldığında otomatik pozlama oturana kadar kareler karanlık gelebilir.

### 5.6 Dosyayı Windows'a alma

| Komut | Nerede | Ne yapar |
|---|---|---|
| `hostname -I` | Pi | Pi'nin IP adreslerini gösterir (bizde `172.20.10.3`). |
| `scp pi2@172.20.10.3:~/ros_test.jpg $HOME\Desktop\` | Windows PowerShell | Dosyayı SSH üzerinden Pi'den masaüstüne kopyalar. |

Hata dersi: İlk denemede IP `172.10.3` yazıldı. Windows bunu `172.10.0.3` olarak okuyup başka bir adrese bağlanmaya çalıştı. IP'nin dört parçasını da yazmak gerekiyor.

---

## 6. Neden çalıştı? (ve neyi kesin bilmiyoruz)

Üç farklı libcamera sürümü denedik:

| Sürüm | Nereden | Sonuç |
|---|---|---|
| 0.2.0 | Ubuntu 24.04 paketi (`/usr`) | `prepareIsp` hatasıyla çöktü |
| 0.7 | ROS deposu (`/opt/ros/jazzy`) | IPA işleminde çöktü + zaman aşımları |
| 0.8.0 | Raspberry Pi çatalı, kendi derlememiz (`~/camera_ws`) | Çalıştı |

Kaynaklardan öğrendiklerimiz:

- 0.2.0'daki hata, libcamera geliştirici listesindeki bir yamayla örtüşüyor. Yama, Pi'nin `vc4` hattında boş bir tampon kimliğinin ilklendirilmeden bırakılmasını düzeltiyor.
- ROS paketindeki hata, ST forumunda anlatılan duruma benziyor: IPA modülü "güvenilmez" sayılınca ayrı bir işlemde (izole modda) çalışıp çöküyor. `camera_ros` geliştiricisi de Raspberry Pi çatalının gerekli olduğunu, çünkü upstream tarafta V4L2'de "gömülü veri akışı" desteğinin eksik olduğunu söylüyor.
- libcamera ile IPA'yı birlikte derlediğimizde ikisi aynı anahtarla imzalanıyor ve izolasyon sorunu çıkmıyor.

**Kesin bilmediğimiz:** İki çökmenin tam kök nedenini kendi sistemimizde kanıtlamadık. Elimizdeki, sağlam kaynaklarla uyumlu açıklamalar. Çalışan yolun Pi çatalının derlenmesi olduğunu ise test ederek gördük.

---

## 7. Başka bir Raspberry Pi'de tekrar yapmak

Şartlar: Ubuntu 24.04, ROS 2 Jazzy kurulu, kamera Pi kapalıyken takılı.

**Önce bu Pi'de çalışan sürümleri not et:**

```
git -C ~/camera_ws/src/libcamera rev-parse HEAD
git -C ~/camera_ws/src/camera_ros rev-parse HEAD
```

**Yeni Pi'de sırayla:**

```
# 1) Kamera satırlarını kontrol et
grep -i -E "camera|dtoverlay" /boot/firmware/config.txt

# 2) Paketler
sudo apt install -y git python3-colcon-common-extensions python3-colcon-meson python3-rosdep \
  meson ninja-build python3-yaml python3-ply python3-jinja2 \
  libyaml-dev libssl-dev libudev-dev libatomic1 libtiff-dev pybind11-dev

# 3) rosdep
sudo rosdep init || true
rosdep update

# 4) Kaynaklar (aynı sürüme sabitlemek için checkout satırları)
mkdir -p ~/camera_ws/src && cd ~/camera_ws/src
git clone https://github.com/raspberrypi/libcamera.git
git clone https://github.com/christianrauch/camera_ros.git
git -C libcamera checkout <libcamera-kodu>
git -C camera_ros checkout <camera_ros-kodu>

# 5) ROS bağımlılıkları
cd ~/camera_ws
source /opt/ros/jazzy/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro jazzy --skip-keys=libcamera

# 6) Derle
colcon build --event-handlers=console_direct+ 2>&1 | tee build.log

# 7) Çalıştır
source ~/camera_ws/install/setup.bash
ros2 run camera_ros camera_node --ros-args -p width:=1280 -p height:=960
```

Sonra başka bir terminalde `kontrol.py` ile parlaklığı doğrula.

**Olası sorunlar:**
- apt'te "unmet dependencies" çakışması: `/etc/apt/sources.list.d/ubuntu.sources` dosyasında `Suites:` satırında `noble-updates` olup olmadığına bak. Bu Pi'de eksikti.
- İzin sorunları: `sudo usermod -aG video $USER` ve oturumu yeniden aç.
- Farklı kamera sensörü: `config.txt`'deki `dtoverlay` satırı değişir.

---

## 8. Sık sorulanlar

**ROS kurarak mı çözdük?**
Hayır. ROS zaten kuruluydu (projen için). Çözüm, ROS'un kendisi değil; Raspberry Pi'nin libcamera çatalını derlemek. ROS'un derleme aracı (`colcon`) ve `camera_ros` bu derlemeyi kolaylaştırdı, o kadar.

**Kamera yalnızca ROS içinde mi çalışıyor?**
Şu an kameraya erişen tek çalışan yol `camera_node`. O, libcamera üzerinden kamerayı açıp görüntüyü ROS topic'ine yayınlıyor. ROS dışındaki programlar (ör. `cv2.VideoCapture(0)`, Cheese) hâlâ ham `/dev/video0`'a gidip siyah görüntü alır. Kendi kodunda görüntüyü almak için `/camera/image_raw` topic'ine abone olursun.

**Sistemdeki `cam` / `qcam` düzeldi mi?**
Hayır. Yeni libcamera yalnızca `~/camera_ws` içinde. Sistemdeki `cam` hâlâ Ubuntu'nun 0.2.0 sürümünü kullanır ve aynı hatayla çöker.

**Kamerayı aynı anda iki program açabilir mi?**
Hayır, libcamera bir kamerayı aynı anda tek uygulamaya verir. `camera_node` çalışırken başka program kamerayı açamaz. Birden fazla programın görüntüyü kullanması gerekiyorsa hepsi aynı ROS topic'ine abone olur.

**Windows bilgisayarımın kendi kamerasıyla ilgisi var mı?**
Hayır. Bu kamera Pi'ye bağlı ve yalnızca Pi'de çalışıyor. Windows'a şimdilik dosya kopyalayarak aldık. Canlı izlemek için Windows'ta ROS 2 kurup aynı ağdan topic'e bağlanmak ya da görüntüyü bir ağ akışı olarak yayınlamak gibi yollar var, bunları henüz denemedik.

---

## 9. Yapılacaklar

1. QR kod çözümleyen ROS düğümü (`/camera/image_raw`'a abone olacak).
2. `~/.bashrc` içine `source ~/camera_ws/install/setup.bash` satırını eklemek.
3. Kameranın bilgisayar açılınca otomatik başlaması (systemd).
4. `sudo reboot` (apt kurulumundan kalan servis yeniden başlatma uyarıları için).
5. `noble-updates` deposunu eklemek (`/etc/apt/sources.list.d/ubuntu.sources`). Şu an yalnızca `noble` ve `noble-security` var, normal güncellemeler gelmiyor.
6. `config.txt` içinde tekrar eden satırları temizlemek (`camera_auto_detect=1` iki kez, `dtoverlay=vc4-kms-v3d` üç kez geçiyor, elle `dtoverlay=imx219` de var).

---

## Kaynaklar

- Ubuntu kamera dokümanı: https://ubuntu.com/hardware/docs/boards/how-to/special_hardware/rpi-camera/
- Raspberry Pi kamera yazılımı: https://www.raspberrypi.com/documentation/accessories/camera-software.html
- libcamera 0.2.0 yaması: https://patchwork.libcamera.org/api/patches/19446/comments
- camera_ros README: https://docs.ros.org/en/jazzy/p/camera_ros/__README.html
- camera_ros geliştiricisinin açıklaması: https://discourse.openrobotics.org/t/installation-and-configuration-of-the-raspberry-pi-camera-on-a-ros-2-jazzy-raspberry-pi-5/45177
- ST forumu (IPA izolasyonu): https://community.st.com/imaging-sensors-49/invalid-signature-error-in-ipa-module-for-vd56g3-150622
