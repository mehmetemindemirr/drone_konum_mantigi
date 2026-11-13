"""tools/ altindaki betiklerin ROS'suz sinanabilen kisimlari."""
import csv
import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tests"))
import synthetic as syn  # noqa: E402


def _load(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


save_frames = _load(ROOT / "tools" / "save_frames.py")


def fake_msg(rgb, encoding="rgb8", pad_bytes=0):
    """Gercek Image mesajini taklit eder; satir sonuna dolgu baytlari ekleyebilir."""
    h, w = rgb.shape[:2]
    ch = 1 if rgb.ndim == 2 else rgb.shape[2]
    rows = rgb.reshape(h, w * ch)
    if pad_bytes:
        rows = np.hstack([rows, np.zeros((h, pad_bytes), np.uint8)])
    return SimpleNamespace(height=h, width=w, encoding=encoding,
                           step=rows.shape[1], data=rows.tobytes())


class ImageDecodeTests(unittest.TestCase):
    def test_rgb8_is_converted_to_bgr(self):
        bgr = syn.disc(syn.blank(64, 48), (32, 24), 10, syn.RED_BGR)
        msg = fake_msg(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), "rgb8")
        self.assertTrue(np.array_equal(save_frames.image_msg_to_bgr(msg), bgr))

    def test_bgr8_passthrough(self):
        bgr = syn.disc(syn.blank(64, 48), (32, 24), 10, syn.BLUE_BGR)
        self.assertTrue(np.array_equal(save_frames.image_msg_to_bgr(fake_msg(bgr, "bgr8")), bgr))

    def test_row_padding_is_dropped(self):
        bgr = syn.disc(syn.blank(63, 47), (30, 20), 9, syn.RED_BGR)
        msg = fake_msg(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), "rgb8", pad_bytes=5)
        out = save_frames.image_msg_to_bgr(msg)
        self.assertEqual(out.shape, (47, 63, 3))
        self.assertTrue(np.array_equal(out, bgr))

    def test_mono8(self):
        gray = np.arange(48 * 64, dtype=np.uint32).reshape(48, 64).astype(np.uint8)
        out = save_frames.image_msg_to_bgr(fake_msg(gray, "mono8"))
        self.assertEqual(out.shape, (48, 64, 3))
        self.assertTrue(np.array_equal(out[:, :, 0], gray))

    def test_unsupported_encoding_raises(self):
        msg = SimpleNamespace(height=2, width=2, encoding="16UC1", step=4, data=bytes(8))
        with self.assertRaises(ValueError):
            save_frames.image_msg_to_bgr(msg)

    def test_yaw_enu_convention(self):
        # ENU'da yaw 0 = Dogu; z ekseninde 90 derece donus quaternion (0,0,.7071,.7071)
        self.assertAlmostEqual(save_frames.quaternion_to_yaw_deg(0, 0, 0, 1), 0.0)
        self.assertAlmostEqual(save_frames.quaternion_to_yaw_deg(0, 0, 0.70710678, 0.70710678), 90.0, 4)


class DetectImagesCliTests(unittest.TestCase):
    def test_end_to_end_with_pose_hint(self):
        fx = 381.4                       # 640 px, 80 derece
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            # kare_0000: 15 m irtifada beklenen yaricap ~25 px; goruntude de ~25 px -> kabul
            # kare_0001: ayni irtifa kaydi ama goruntude 70 px'lik disk -> ipucu reddeder
            # kare_0002: bos sahne
            cv2.imwrite(str(tmp / "kare_0000.png"),
                        syn.disc(syn.blank(), (320, 240), 25, syn.RED_BGR))
            cv2.imwrite(str(tmp / "kare_0001.png"),
                        syn.disc(syn.blank(), (320, 240), 70, syn.RED_BGR))
            cv2.imwrite(str(tmp / "kare_0002.png"), syn.blank())
            with open(tmp / "kareler.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["dosya", "wall_t", "stamp_s", "x", "y", "z", "yaw_enu_deg", "poz_yasi_s"])
                for i in range(3):
                    w.writerow([f"kare_{i:04d}.png", "0", "0", "0", "0", "15.0", "0", "0.1"])

            out_dir = tmp / "sonuc"
            res = subprocess.run(
                [sys.executable, str(ROOT / "tools" / "detect_images.py"), str(tmp),
                 "--out", str(out_dir), "--poses", str(tmp / "kareler.csv"), "--fx", str(fx)],
                capture_output=True, text=True)
            self.assertEqual(res.returncode, 0, res.stderr)
            lines = res.stdout.splitlines()
            by_name = {}
            for i, ln in enumerate(lines):
                if ln.startswith("kare_"):
                    by_name[ln.split(":")[0]] = ln
            self.assertIn("1 tespit", by_name["kare_0000.png"])
            self.assertIn("0 tespit", by_name["kare_0001.png"])
            self.assertIn("0 tespit", by_name["kare_0002.png"])
            self.assertIn("1/3", res.stdout)
            # isaretli kopyalar yazildi, tespit olan karenin kopyasi orijinalden farkli
            self.assertTrue((out_dir / "kare_0000.png").exists())
            a = cv2.imread(str(tmp / "kare_0000.png"))
            b = cv2.imread(str(out_dir / "kare_0000.png"))
            self.assertFalse(np.array_equal(a, b))

    def test_missing_input_returns_error_code(self):
        with tempfile.TemporaryDirectory() as tmp:
            res = subprocess.run([sys.executable, str(ROOT / "tools" / "detect_images.py"), tmp],
                                 capture_output=True, text=True)
            self.assertEqual(res.returncode, 1)

    def test_poses_without_fx_is_usage_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            cv2.imwrite(os.path.join(tmp, "a.png"), syn.blank())
            res = subprocess.run([sys.executable, str(ROOT / "tools" / "detect_images.py"), tmp,
                                  "--poses", os.path.join(tmp, "x.csv")],
                                 capture_output=True, text=True)
            self.assertEqual(res.returncode, 2)


if __name__ == "__main__":
    unittest.main()
