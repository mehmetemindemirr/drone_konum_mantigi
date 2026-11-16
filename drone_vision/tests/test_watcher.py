"""PadWatcher (gozlemci) ve live_detect kurulumu: ROS'suz, sahte saat ile."""
import importlib.util
import os
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

import synthetic as syn  # noqa: E402
from adapters.pad_watcher import PadWatcher  # noqa: E402
from pad_vision import ColorCoverage, HsvPadDetector  # noqa: E402


class Clock:
    def __init__(self): self.t = 100.0
    def __call__(self): return self.t


def make(**kw):
    logs, clk = [], Clock()
    w = PadWatcher(HsvPadDetector(), ColorCoverage(), log=logs.append, clock=clk, **kw)
    return w, logs, clk


def feed(w, img, enc="bgr8"):
    w.on_image(syn.msg_from_bgr(img, enc))


class WatcherTests(unittest.TestCase):
    def test_announces_then_loses(self):
        w, logs, clk = make(every_s=0.5)
        feed(w, syn.disc(syn.blank(), (320, 240), 60, syn.RED_BGR))
        rep = w.update(10.0)
        self.assertEqual(len(rep.detections), 1)
        self.assertTrue(any("KIRMIZI gorundu" in l for l in logs))
        for _ in range(3):
            clk.t += 1.0
            feed(w, syn.blank())
            w.update(10.0)
        self.assertTrue(any("KIRMIZI kayboldu" in l for l in logs))

    def test_full_red_reports_coverage_not_detection(self):
        w, logs, _ = make()
        feed(w, syn.blank(bgr=syn.RED_BGR))
        rep = w.update(2.0)
        self.assertEqual(rep.detections, ())
        self.assertGreater(rep.coverage["kirmizi"], 0.98)
        self.assertTrue(any("KADRAJIN" in l for l in logs))

    def test_throttle(self):
        w, _, clk = make(every_s=0.5)
        feed(w, syn.blank())
        a = w.update(); clk.t += 0.1
        self.assertIs(w.update(), a)           # yeni islem yok
        clk.t += 0.5
        self.assertIsNot(w.update(), a)

    def test_no_image_and_stale_warn_but_return_none(self):
        w, logs, clk = make(max_frame_age_s=2.0)
        self.assertIsNone(w.update())
        self.assertTrue(any("henuz goruntu" in l for l in logs))
        feed(w, syn.blank()); clk.t += 5.0
        self.assertIsNone(w.update())
        self.assertTrue(any("guncellenmedi" in l for l in logs))

    def test_bad_message_never_raises(self):
        w, logs, _ = make()
        w.on_image(syn.fake_msg(syn.blank()[:, :, 0], "16UC1"))
        self.assertIsNone(w.update())
        self.assertTrue(any("hata" in l for l in logs))

    def test_encodings_give_same_detection(self):
        img = syn.disc(syn.blank(), (320, 240), 60, syn.RED_BGR)
        for enc in ("bgr8", "rgb8", "bgra8", "rgba8"):
            with self.subTest(enc=enc):
                w, _, _ = make()
                feed(w, img, enc)
                self.assertEqual(len(w.update(10.0).detections), 1)

    def test_swap_rb_fixes_mislabeled_channels(self):
        img = syn.disc(syn.blank(), (320, 240), 60, syn.RED_BGR)
        w, _, _ = make(swap_rb=True)
        w.on_image(syn.fake_msg(img, "bgr8"))     # bgr dizi ama "rgb8" gibi okunacak sekilde
        w._latest = (syn.fake_msg(img, "rgb8"), w._latest[1])  # yanlis etiket
        self.assertEqual(w.update(10.0).detections[0].color, "kirmizi")

    def test_hint_rejects_wrong_size(self):
        w, _, _ = make(hfov_deg=80.0, use_hint=True)
        feed(w, syn.disc(syn.blank(), (320, 240), 100, syn.RED_BGR))   # 15 m'de ~25 px beklenir
        self.assertEqual(w.update(15.0).detections, ())

    def test_fx_estimate_logged_high_enough(self):
        w, logs, _ = make()
        feed(w, syn.disc(syn.blank(), (320, 240), 38, syn.RED_BGR))
        w.update(10.0)
        self.assertTrue(any("olculen fx~" in l for l in logs))

    def test_use_hint_requires_hfov(self):
        with self.assertRaises(ValueError):
            PadWatcher(HsvPadDetector(), use_hint=True)


class LiveDetectSetup(unittest.TestCase):
    def test_build_watcher_and_args(self):
        spec = importlib.util.spec_from_file_location("live_detect", ROOT / "tools" / "live_detect.py")
        m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
        args = m.parse(["--topic", "/camera/image_raw", "--hfov", "62.2", "--use-hint"])
        w = m.build_watcher(args, log=lambda s: None)
        feed(w, syn.disc(syn.blank(1280, 960), (640, 480), 120, syn.RED_BGR))
        self.assertEqual(len(w.update(10.0).detections), 1)
        with self.assertRaises(SystemExit):
            m.parse(["--use-hint"])


if __name__ == "__main__":
    unittest.main()
