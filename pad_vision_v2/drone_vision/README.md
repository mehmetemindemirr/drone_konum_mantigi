# drone_vision: pad tespit modülü (v1)

Renkli (kırmızı/mavi) daire şeklindeki iniş pad'ini kameradan bulur. Yalnızca **OpenCV + NumPy** kullanır,
ROS'u bilmez. Ana görev onu gerektiğinde çağırır; kendi başına abonelik, zamanlayıcı, global durum yoktur.

## Yapı

```
pad_vision/
  models.py        veri tipleri + ayarlar (sadece standart kütüphane): PadDetection, DetectorConfig, ...
  segmentation.py  ColorSegmenter : HSV + renk profili  -> ikili maske
  analysis.py      ShapeAnalyzer  : maske              -> aday blob'lar + şekil ölçüleri (karar vermez)
  detector.py      PadDetector (arayüz) + HsvPadDetector (somut) : adayları süzer, puanlar
  viz.py           draw_detections : hata ayıklama çizimi
tools/
  save_frames.py   [ROS adaptörü] topic'ten PNG + drone konumu CSV kaydeder
  detect_images.py [ROS'suz] klasördeki kareler üzerinde dedektörü çalıştırır, işaretli kopya yazar
tests/             33 birim testi (sentetik sahneler)
```

SOLID eşleşmesi: her sınıfın tek işi var (S); yeni renk = yeni `ColorProfile`, yeni dedektör = `PadDetector`'u
gerçekleyen yeni sınıf, mevcut kod değişmez (O); `HsvPadDetector` parçalarını yapıcıdan alır, test bunu kullanır (D);
ana görev yalnızca `PadDetector` arayüzünü ve `PadDetection` tipini bilir (D, I).

## Hızlı başlangıç

```bash
cd drone_vision
python3 -m unittest discover -s tests          # önce kendi makinende 33 testin geçtiğini gör
```

Kare toplamak (Gazebo + SITL + uçuş kodu çalışırken, ROS yüklü terminalde):

```bash
source /opt/ros/humble/setup.bash
python3 tools/save_frames.py --out kareler --every 1.0 --max 120
```

Toplanan karelerde tespit (işaretli kopyalar `sonuc/` içine yazılır):

```bash
python3 tools/detect_images.py kareler --out sonuc
python3 tools/detect_images.py kareler --out sonuc --poses kareler/kareler.csv --fx 190.7   # boyut ipucuyla
```

`--fx`: kullandığın çözünürlükteki odak uzaklığı. 640x480 / 80° için 381.4, 320x240 / 80° için 190.7.

## Ana görevde kullanım

```python
from pad_vision import HsvPadDetector, DetectionHint, expected_radius_px

detector = HsvPadDetector()                                   # bir kez oluştur

# ana görev, ihtiyaç duyduğu anda (frame_bgr: (H, W, 3) uint8 BGR):
hint = DetectionHint(expected_radius_px(altitude_m, fx_px=190.7))   # isteğe bağlı
dets = detector.detect(frame_bgr, hint)
red = [d for d in dets if d.color == "kirmizi" and not d.touches_border]
if red:
    u, v = red[0].center_px          # orijinal görüntü pikseli
```

## Ayarlanacak eşikler (`DetectorConfig`, `models.py`)

| Alan | Ne yapar | Ne zaman değiştir |
|---|---|---|
| `RED` / `BLUE` HSV aralıkları | hangi piksel "kırmızı" | gerçek kamerada ilk iş: pad kaçırılıyorsa S/V alt sınırını düşür |
| `min_radius_px` | en küçük kabul edilen pad | yüksekten (30 m+) tespit gerekiyorsa düşür |
| `min_circularity`, `min_ellipse_iou`, `max_rect_fill` | şekil filtreleri | gerçek pad eleniyorsa gevşet, yanlış pozitif varsa sıkılaştır |
| `size_tolerance` | boyut ipucu toleransı | irtifa ölçümün gürültülüyse artır |
| `min_confidence` | genel eşik | son çare |

## Bilinen sınırlar (dürüst liste)

- Eşikler **sentetik sahnelerle** ayarlandı. Gazebo ve gerçek kamera karelerinde yeniden ayarlanması beklenir.
- Pad kadrajdan yarıdan fazla taşarsa reddedilir (merkez önyargılı olur). Son birkaç metrede bu kaçınılmaz;
  inişin o kısmı son bilinen merkeze göre yapılmalı.
- Düzgün altıgen/sekizgen dairelerden ayırt edilemez (şekilleri neredeyse daire). Boyut ipucu ve ardışık kare
  onayı (bu modülde YOK, bir sonraki katman) bunu telafi eder.
- `confidence` kalibre edilmiş bir olasılık değil, **sıralama puanı**dır.
- Süre: bu geliştirme makinesinde 640x480'de ~0,8 ms/kare. Raspberry Pi'de ölçülmedi.
- Test ortamı: Python 3.13 + OpenCV 5.0. Hedef (Python 3.10, OpenCV 4.x) için sözdizimi doğrulandı, ama
  çalışma doğrulaması senin ilk `unittest` çalıştırmanla olacak.

## Canli gozlem (Nov 15)
- `python3 tools/live_detect.py --topic /drone/down_camera/image_raw --hfov 80 --use-hint [--publish|--show]`
  ayri surecte kirmizi/mavi pad'i canli yazar. Gercek kamera: `--topic /camera/image_raw --hfov 62.2`.
- `yaw_entegrasyon.md`: yaw.py'ye 4 satirlik PadWatcher ekleme.
- Tam kirmizi kare: dedektor DAIRE sekli arar, bu yuzden tespit yok; bunun yerine
  `KADRAJIN %X'I KIRMIZI` (ColorCoverage) uyarisi verir.
- `guven` dogruluk yuzdesi degil, siralama skorudur.
