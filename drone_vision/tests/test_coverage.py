"""ColorCoverage ve "kipkirmizi kare" davranisi testleri."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import synthetic as syn  # noqa: E402
from pad_vision import RED, ColorCoverage, HsvPadDetector  # noqa: E402
from pad_vision.imaging import downscale_to_width, ensure_bgr  # noqa: E402
from pad_vision.segmentation import ColorSegmenter  # noqa: E402


class CoverageTests(unittest.TestCase):
    def setUp(self):
        self.cov = ColorCoverage()

    def test_full_red_frame(self):
        img = syn.blank(bgr=syn.RED_BGR)
        c = self.cov.measure(img)
        self.assertGreater(c["kirmizi"], 0.98)
        self.assertEqual(c["mavi"], 0.0)

    def test_half_red_frame(self):
        img = syn.blank()
        img[:, :320] = syn.RED_BGR
        self.assertAlmostEqual(self.cov.measure(img)["kirmizi"], 0.5, delta=0.02)

    def test_empty_scene(self):
        self.assertEqual(self.cov.measure(syn.blank()), {"kirmizi": 0.0, "mavi": 0.0})

    def test_disc_fraction_matches_geometry(self):
        img = syn.disc(syn.blank(), (320, 240), 50, syn.RED_BGR)
        expected = math.pi * 50 ** 2 / (640 * 480)
        self.assertAlmostEqual(self.cov.measure(img)["kirmizi"], expected, delta=0.006)

    def test_two_colors_reported_separately(self):
        img = syn.blank()
        img[:, :160] = syn.RED_BGR
        img[:, 480:] = syn.BLUE_BGR
        c = self.cov.measure(img)
        self.assertAlmostEqual(c["kirmizi"], 0.25, delta=0.02)
        self.assertAlmostEqual(c["mavi"], 0.25, delta=0.02)

    def test_any_resolution(self):
        for w, h in ((160, 120), (640, 480), (1280, 960)):
            with self.subTest(cozunurluk=(w, h)):
                img = syn.blank(w, h, bgr=syn.RED_BGR)
                self.assertGreater(self.cov.measure(img)["kirmizi"], 0.98)

    def test_invalid_input_raises(self):
        for bad in (np.zeros((10, 10), np.uint8), np.zeros((10, 10, 3), np.float32), None):
            with self.assertRaises(ValueError):
                self.cov.measure(bad)

    def test_injected_segmenter_is_used(self):
        class Blank(ColorSegmenter):
            def mask(self, hsv, profile):
                return np.zeros(hsv.shape[:2], np.uint8)
        c = ColorCoverage(segmenter=Blank()).measure(syn.blank(bgr=syn.RED_BGR))
        self.assertEqual(c["kirmizi"], 0.0)

    def test_profile_subset(self):
        self.assertEqual(list(ColorCoverage(profiles=(RED,)).measure(syn.blank())), ["kirmizi"])


class FullFrameDesignTests(unittest.TestCase):
    """Dedektor tam kirmizi kareyi 'pad merkezi' diye UYDURMAZ; kaplama sinyali bunu soyler."""

    def test_full_red_frame_has_no_circle_but_high_coverage(self):
        img = syn.blank(bgr=syn.RED_BGR)
        self.assertEqual(HsvPadDetector().detect(img), [])
        self.assertGreater(ColorCoverage().measure(img)["kirmizi"], 0.98)

    def test_oversized_pad_is_never_reported_without_border_flag(self):
        """Pad kadraja sigmiyorsa: ya hic tespit yok ya da 'kenarda' bayragiyla tespit var.

        Hafif tasan pad kabul edilir (simetrik tasmada merkez dogru kalir) ama merkezi
        onyargili olabilecegi icin ASLA bayraksiz verilmez. Tam kaplayan kare reddedilir.
        """
        det, cov = HsvPadDetector(), ColorCoverage()
        for r in (260, 300, 340, 380, 420, 600, 900):
            with self.subTest(yaricap=r):
                img = syn.disc(syn.blank(), (320, 240), r, syn.RED_BGR)
                for d in det.detect(img):
                    self.assertTrue(d.touches_border, f"r={r}: bayraksiz tespit")
                self.assertGreater(cov.measure(img)["kirmizi"], 0.5)

    def test_covering_disc_gives_no_detection(self):
        # Koseler dahil tum kadraji kaplayan disk: sekil kalmaz -> tespit yok, kaplama ~1
        img = syn.disc(syn.blank(), (320, 240), 600, syn.RED_BGR)
        self.assertEqual(HsvPadDetector().detect(img), [])
        self.assertGreater(ColorCoverage().measure(img)["kirmizi"], 0.98)


class ImagingTests(unittest.TestCase):
    def test_downscale_keeps_aspect_and_skips_small(self):
        big = syn.blank(1280, 960)
        self.assertEqual(downscale_to_width(big, 320).shape[:2], (240, 320))
        small = syn.blank(100, 80)
        self.assertIs(downscale_to_width(small, 320), small)

    def test_ensure_bgr_accepts_valid(self):
        ensure_bgr(syn.blank(10, 10))


if __name__ == "__main__":
    unittest.main()
