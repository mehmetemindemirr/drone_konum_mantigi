"""pad_vision birim testleri.  Calistirma:  python3 -m unittest discover -s tests -v"""
import math
import os
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import synthetic as syn  # noqa: E402
from pad_vision import (BLUE, RED, DetectionHint, DetectorConfig, HsvPadDetector,  # noqa: E402
                        draw_detections, expected_radius_px)
from pad_vision.segmentation import ColorSegmenter  # noqa: E402


class DetectorTests(unittest.TestCase):
    def setUp(self):
        self.det = HsvPadDetector()

    # ---------------- dogru tespit ----------------
    def test_red_disc_center_and_radius(self):
        img = syn.disc(syn.blank(), (400, 200), 50, syn.RED_BGR)
        res = self.det.detect(img)
        self.assertEqual(len(res), 1)
        d = res[0]
        self.assertEqual(d.color, "kirmizi")
        self.assertLess(math.hypot(d.center_px[0] - 400, d.center_px[1] - 200), 1.0)
        self.assertLess(abs(d.radius_px - 50), 3.0)
        self.assertFalse(d.touches_border)
        self.assertGreater(d.confidence, 0.85)

    def test_blue_disc(self):
        img = syn.disc(syn.blank(), (250, 300), 45, syn.BLUE_BGR)
        res = self.det.detect(img)
        self.assertEqual([d.color for d in res], ["mavi"])
        self.assertLess(math.hypot(res[0].center_px[0] - 250, res[0].center_px[1] - 300), 1.0)

    def test_both_colors_in_one_frame(self):
        img = syn.blank()
        syn.disc(img, (150, 150), 40, syn.RED_BGR)
        syn.disc(img, (450, 300), 60, syn.BLUE_BGR)
        res = self.det.detect(img)
        self.assertEqual({d.color for d in res}, {"kirmizi", "mavi"})

    def test_results_sorted_by_confidence(self):
        img = syn.blank()
        syn.disc(img, (150, 150), 40, syn.RED_BGR)
        syn.disc(img, (450, 300), 60, syn.BLUE_BGR, squash=0.6, angle=20)
        res = self.det.detect(img)
        confs = [d.confidence for d in res]
        self.assertEqual(confs, sorted(confs, reverse=True))

    def test_empty_scene(self):
        self.assertEqual(self.det.detect(syn.blank()), [])

    # ---------------- cozunurluk bagimsizligi ----------------
    def test_coordinates_in_original_resolution(self):
        img = syn.disc(syn.blank(1280, 960), (800, 400), 100, syn.RED_BGR)
        d = self.det.detect(img)[0]
        self.assertLess(math.hypot(d.center_px[0] - 800, d.center_px[1] - 400), 1.5)
        self.assertLess(abs(d.radius_px - 100), 5.0)

    def test_small_image_not_upscaled(self):
        img = syn.disc(syn.blank(320, 240), (160, 120), 30, syn.RED_BGR)
        d = self.det.detect(img)[0]
        self.assertLess(math.hypot(d.center_px[0] - 160, d.center_px[1] - 120), 1.0)

    # ---------------- yanlis pozitifler ----------------
    def test_rejects_non_disc_shapes(self):
        shapes = {
            "dikdortgen": lambda im: syn.rect(im, 200, 200, 360, 260, syn.RED_BGR),
            "kare": lambda im: syn.rect(im, 200, 150, 300, 250, syn.RED_BGR),
            "ucgen": lambda im: syn.polygon(im, [(300, 120), (380, 260), (220, 260)], syn.RED_BGR),
            "c_halka": lambda im: cv2.ellipse(im, (300, 200), (60, 60), 0, 40, 320,
                                              syn.RED_BGR, 28, cv2.LINE_AA),
        }
        for name, draw in shapes.items():
            with self.subTest(sekil=name):
                img = syn.blank()
                draw(img)
                self.assertEqual(self.det.detect(img), [], f"{name} yanlis pozitif verdi")

    def test_rejects_tiny_speck(self):
        img = syn.disc(syn.blank(), (300, 200), 3, syn.RED_BGR)
        self.assertEqual(self.det.detect(img), [])

    def test_rejects_wrong_colors(self):
        for name, bgr in {"turuncu": (0, 128, 255), "soluk_pembe": (180, 180, 255),
                          "cok_koyu_kirmizi": (0, 0, 50)}.items():
            with self.subTest(renk=name):
                img = syn.disc(syn.blank(), (300, 200), 50, bgr)
                self.assertEqual(self.det.detect(img), [])

    # ---------------- gercekci bozulmalar ----------------
    def test_tilted_pad_is_detected(self):
        img = syn.disc(syn.blank(), (320, 240), 60, syn.RED_BGR, squash=0.7, angle=30)
        res = self.det.detect(img)
        self.assertEqual(len(res), 1)
        self.assertLess(math.hypot(res[0].center_px[0] - 320, res[0].center_px[1] - 240), 1.5)

    def test_noise_and_blur(self):
        img = syn.disc(syn.blank(), (400, 200), 50, syn.RED_BGR)
        img = cv2.GaussianBlur(img, (5, 5), 1.5)
        img = syn.add_noise(img, 8.0, seed=1)
        res = self.det.detect(img)
        self.assertEqual(len(res), 1)
        self.assertLess(math.hypot(res[0].center_px[0] - 400, res[0].center_px[1] - 200), 2.0)

    def test_dimmer_lighting(self):
        img = syn.dim(syn.disc(syn.blank(), (400, 200), 50, syn.RED_BGR), 0.5)
        self.assertEqual(len(self.det.detect(img)), 1)

    # ---------------- kenar ----------------
    def test_slightly_clipped_pad_is_flagged(self):
        img = syn.disc(syn.blank(), (30, 200), 50, syn.RED_BGR)      # 20 px disari tasiyor
        res = self.det.detect(img)
        self.assertEqual(len(res), 1)
        self.assertTrue(res[0].touches_border)
        full = self.det.detect(syn.disc(syn.blank(), (200, 200), 50, syn.RED_BGR))[0]
        self.assertLess(res[0].confidence, full.confidence)

    def test_half_visible_pad_is_rejected(self):
        img = syn.disc(syn.blank(), (0, 200), 60, syn.RED_BGR)       # yarisi kadraj disinda
        self.assertEqual(self.det.detect(img), [])

    # ---------------- boyut ipucu ----------------
    def test_size_hint_accepts_matching_and_rejects_mismatch(self):
        img = syn.disc(syn.blank(), (400, 200), 50, syn.RED_BGR)
        ok = self.det.detect(img, DetectionHint(expected_radius_px=52.0))
        self.assertEqual(len(ok), 1)
        bad = self.det.detect(img, DetectionHint(expected_radius_px=15.0))
        self.assertEqual(bad, [])

    def test_hint_none_values_are_ignored(self):
        img = syn.disc(syn.blank(), (400, 200), 50, syn.RED_BGR)
        self.assertEqual(len(self.det.detect(img, DetectionHint())), 1)
        self.assertEqual(len(self.det.detect(img, DetectionHint(expected_radius_px=0.0))), 1)

    # ---------------- girdi dogrulama / yan etki ----------------
    def test_invalid_input_raises(self):
        for bad in (np.zeros((100, 100), np.uint8),
                    np.zeros((100, 100, 3), np.float32),
                    np.zeros((100, 100, 4), np.uint8),
                    "resim.png", None):
            with self.subTest(girdi=type(bad).__name__):
                with self.assertRaises(ValueError):
                    self.det.detect(bad)

    def test_input_is_not_modified(self):
        img = syn.disc(syn.blank(), (400, 200), 50, syn.RED_BGR)
        before = img.copy()
        res = self.det.detect(img)
        out = draw_detections(img, res)
        self.assertTrue(np.array_equal(img, before))
        self.assertFalse(np.array_equal(out, before))

    # ---------------- bagimlilik enjeksiyonu / yapilandirma ----------------
    def test_injected_segmenter_is_used(self):
        class BlankSegmenter(ColorSegmenter):
            def mask(self, hsv, profile):
                return np.zeros(hsv.shape[:2], np.uint8)

        det = HsvPadDetector(segmenter=BlankSegmenter())
        img = syn.disc(syn.blank(), (400, 200), 50, syn.RED_BGR)
        self.assertEqual(det.detect(img), [])

    def test_single_profile_only(self):
        img = syn.blank()
        syn.disc(img, (150, 150), 40, syn.RED_BGR)
        syn.disc(img, (450, 300), 60, syn.BLUE_BGR)
        res = HsvPadDetector(profiles=(RED,)).detect(img)
        self.assertEqual([d.color for d in res], ["kirmizi"])

    def test_config_threshold_is_respected(self):
        img = syn.disc(syn.blank(), (400, 200), 50, syn.RED_BGR)
        strict = HsvPadDetector(config=DetectorConfig(min_confidence=0.999))
        self.assertEqual(strict.detect(img), [])


class HelperTests(unittest.TestCase):
    def test_expected_radius(self):
        # fx=381.4 (640 px, 80 derece), 15 m, 1 m yaricap -> ~25.4 px
        self.assertAlmostEqual(expected_radius_px(15.0, 381.4), 25.43, places=1)

    def test_expected_radius_rejects_bad_altitude(self):
        for bad in (0.0, -3.0):
            with self.assertRaises(ValueError):
                expected_radius_px(bad, 381.4)


if __name__ == "__main__":
    unittest.main()
