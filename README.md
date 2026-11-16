# drone_vision: kırmızı/mavi iniş pedi tespiti

Aşağıya bakan kameradan 1 m yarıçaplı kırmızı/mavi daire pad'i bulur. Karar vermez, uçuşa karışmaz:
sadece görür ve bildirir. ROS'a bağımlı olan tek yer `adapters/` ve `tools/`; `pad_vision/` saf Python + OpenCV + NumPy.

Durum: simülasyonda (Gazebo Classic + ArduPilot + MAVROS) doğrulandı, ölçülen fx ≈ 385 px (kuramsal 381). Gerçek kamera henüz denenmedi.

## Yapı
| Yol | Görev |
|---|---|
| `pad_vision/` | Tespit çekirdeği: HSV maske → şekil analizi → güven skoru; `coverage.py` kadrajın kaç %'i kırmızı/mavi |
| `adapters/ros_image.py` | ROS görüntü mesajı → OpenCV karesi (cv_bridge yok) |
| `adapters/pad_watcher.py` | Gözlemci: en son kareyi alır, tespit eder, konsola yazar, fx'i ölçer, hata olsa uçuşu bozmaz |
| `tools/live_detect.py` | Canlı izleme (ayrı süreç) |
| `tools/save_frames.py` | Kamera + konumu kaydeder (PNG + CSV) |
| `tools/detect_images.py` | Kayıtlı karelerde çevrimdışı tespit (ROS gerekmez) |
| `tests/` | 58 test, ROS gerekmez |

## Kurulum
Gerekenler: `python3`, `python3-opencv`, `python3-numpy`; canlı araçlar için ROS 2 (Humble: simülasyon, Jazzy: Pi).
Testler: `python3 -m unittest discover -s tests` (hepsi geçmeli).

## Simülasyonda çalıştırma (4 terminal, her birinde `source /opt/ros/humble/setup.bash`)
1. Gazebo → 2. `sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map -I0` → 3. `python3 yaw.py` (MAVROS'u kendisi başlatır)
4. `python3 tools/live_detect.py --topic /drone/down_camera/image_raw --hfov 80 --use-hint --publish`
(Görüntü: `rqt_image_view` → `/pad_vision/debug_image`)

## Raspberry Pi + gerçek kamera (drone yok)
Kamera ROS topic'i olarak yayınlanmalı (`camera_ros`, öneri 640x480; 1280x960 Pi'yi yorar, 640x480 sensör kırpması yapabilir — `kamera-kurulum-notlari.md`'ye bak).
1. `source /opt/ros/jazzy/setup.bash`
2. `python3 tools/live_detect.py --topic /camera/image_raw --hfov 62.2` (**`--use-hint` verme**: drone/irtifa yok)
3. Ekranda tam kırmızı bir daire göster (siyah/gri zemin, parlaklık düşük, ekrana dik, yarıçap ≥ ~20–30 px).
4. Bulamazsa: `python3 tools/save_frames.py --out kareler --every 1 --max 20 --image-topic /camera/image_raw`
   sonra `python3 tools/detect_images.py kareler --out sonuc` ve `sonuc/` içindeki işaretli kareye bak.
5. Renkler ters çıkıyorsa `--swap-rb`. Kırmızı hiç bulunmuyorsa HSV aralığı (aşağıda) ayarlanır.
Pi Camera V2 **NoIR**'da kızılötesi filtre yok: renkler kayar, HSV yeniden ayarlanması beklenir.
Başsız (ekransız) Pi: `--publish` + başka bilgisayarda `rqt_image_view`, veya kareleri kaydedip bilgisayarda incele.

## Log nasıl okunur
- `KIRMIZI gorundu/goruluyor/kayboldu`: merkez (px), yarıçap (px), `guven`, irtifa.
- `guven` doğruluk yüzdesi değil, aday **sıralama skoru**dur.
- `KADRAJ KENARINDA`: pad kesik, merkezi güvenilmez; kaydetme.
- `KADRAJIN %X'I KIRMIZI`: pad kadraja sığmıyor, tam üstündeyiz (tam kırmızı karede daire aranmaz, bu bilerek).
- `olculen fx~…`: gerçek kameranın odak uzaklığı (fx = r_px·irtifa/R); `--hfov` değerini doğrulamak için bak.

## Ana göreve (yaw.py) ekleme: en son, canlı araç sorunsuz çalıştıktan sonra
```python
import sys; sys.path.insert(0, '/YOL/drone_vision')
from adapters.pad_watcher import PadWatcher
from adapters.ros_image import subscribe_image
from pad_vision import HsvPadDetector, ColorCoverage, DetectorConfig
# __init__:
self.pad = PadWatcher(HsvPadDetector(config=DetectorConfig(work_width=320)), ColorCoverage(),
                      hfov_deg=80.0, use_hint=True, every_s=1.0)
subscribe_image(self, '/drone/down_camera/image_raw', self.pad.on_image)
# tick() başında:
self.pad.update(self.rel_alt())
```
Pi'de görüntüyü ana süreçte işlemek `spin_once` döngüsünü yavaşlatabilir; önce ayrı süreç olarak kullan.

## Ayarlanacak eşikler (`DetectorConfig`, `models.py`)
| Alan | Ne zaman değiştir |
|---|---|
| `RED` / `BLUE` HSV aralıkları | gerçek kamerada ilk iş: pad kaçırılıyorsa S/V alt sınırını düşür |
| `min_radius_px` | yüksekten (30 m+) tespit gerekiyorsa düşür |
| `min_circularity`, `min_ellipse_iou`, `max_rect_fill` | gerçek pad eleniyorsa gevşet, yanlış pozitif varsa sıkılaştır |
| `size_tolerance` | irtifa ölçümü gürültülüyse artır |
| `min_confidence` | son çare |

## Bilinen sınırlar
- Eşikler sentetik sahne + Gazebo ile ayarlı; gerçek kamerada yeniden ayar beklenir.
- Pad kadrajdan fazla taşarsa reddedilir; son metreler son bilinen merkeze göre inilmeli.
- Düzgün altıgen/sekizgen daireden ayırt edilemez: ardışık kare onayı (sonraki katman) gerekir.
- Görüntü ve konum zamanları farklı saatlerden gelir (Gazebo sim zamanı / MAVROS duvar saati); konum kaydında `poz_yasi_s` filtrelenmeli.
- Eksenler: MAVROS yerel ENU (yaw 0° = Doğu). Gazebo +x = Kuzey = MAVROS +y.

## Sonraki adımlar
Ardışık kare onayı → piksel→yerel x,y→enlem-boylam + konum kaydı → hassas iniş → sürü paylaşımı.
