# yaw.py'ye PadWatcher ekleme (4 yer, gozlemci: ucusu etkilemez)

1) Importlar (dosyanin basi, `import rclpy` civari):
```python
import sys
sys.path.insert(0, '/YOL/drone_vision')          # drone_vision klasorunun yolu
from adapters.pad_watcher import PadWatcher
from adapters.ros_image import subscribe_image
from pad_vision import HsvPadDetector, ColorCoverage, DetectorConfig
```
2) Sabitler (TARGET_ALT yanina):
```python
KAMERA_TOPIC = '/drone/down_camera/image_raw'    # gercek: '/camera/image_raw'
KAMERA_HFOV  = 80.0                              # gercek: olcumle dogrula (~62.2 tahmin)
```
3) `__init__` icinde (abonelikler yanina):
```python
self.pad = PadWatcher(HsvPadDetector(config=DetectorConfig(work_width=320)),
                      ColorCoverage(), hfov_deg=KAMERA_HFOV, use_hint=True, every_s=1.0)
subscribe_image(self, KAMERA_TOPIC, self.pad.on_image)
```
4) `tick()` icinde en uste (durum makinesinden once):
```python
self.pad.update(self.rel_alt())
```
Ucustan once `python3 tools/live_detect.py` ile AYRI surecte dene; yaw.py icine ancak sonra al.
Pi 4'te 1280x960 goruntuyu ayni surecte islemek spin_once dongusunu yavaslatabilir -> kamerayi
640x480'e dusur veya live_detect'i ayri surec birak.
