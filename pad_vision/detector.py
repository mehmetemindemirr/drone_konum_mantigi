"""Pad dedektoru.

PadDetector  : ana gorevin bagimli oldugu SOYUT arayuz (Protocol).
HsvPadDetector: HSV renk + sekil filtresiyle calisan somut gerceklestirme.

Ana gorev sadece PadDetector'u bilir. Yarin YOLO tabanli bir dedektor yazilirsa,
ana gorev koduna dokunmadan onunla degistirilir.

Bu modul ROS, abonelik, zamanlayici, global durum ICERMEZ. Cagrilinca bir goruntuyu
isler ve sonucu dondurur; baska bir sey yapmaz.
"""
from __future__ import annotations

from typing import List, Optional, Protocol, Sequence

import cv2
import numpy as np

from .analysis import Candidate, ShapeAnalyzer
from .models import BLUE, RED, ColorProfile, DetectionHint, DetectorConfig, PadDetection
from .segmentation import ColorSegmenter


class PadDetector(Protocol):
    def detect(self, image_bgr: np.ndarray,
               hint: Optional[DetectionHint] = None) -> List[PadDetection]:
        """BGR uint8 goruntudeki pad'leri, guvene gore azalan sirada dondurur."""
        ...


class HsvPadDetector:
    def __init__(self,
                 profiles: Sequence[ColorProfile] = (RED, BLUE),
                 config: Optional[DetectorConfig] = None,
                 segmenter: Optional[ColorSegmenter] = None,
                 analyzer: Optional[ShapeAnalyzer] = None):
        self._profiles = tuple(profiles)
        self._cfg = config or DetectorConfig()
        self._segmenter = segmenter or ColorSegmenter(self._cfg.morph_kernel_frac)
        self._analyzer = analyzer or ShapeAnalyzer(self._cfg.min_area_px,
                                                   self._cfg.max_candidates)

    def detect(self, image_bgr: np.ndarray,
               hint: Optional[DetectionHint] = None) -> List[PadDetection]:
        if (not isinstance(image_bgr, np.ndarray) or image_bgr.ndim != 3
                or image_bgr.shape[2] != 3 or image_bgr.dtype != np.uint8):
            raise ValueError("image_bgr: (H, W, 3) uint8 BGR dizisi olmali")

        h0, w0 = image_bgr.shape[:2]
        work = self._downscale(image_bgr)
        sx = work.shape[1] / w0
        sy = work.shape[0] / h0
        hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)

        results: List[PadDetection] = []
        for profile in self._profiles:
            mask = self._segmenter.mask(hsv, profile)
            for cand in self._analyzer.analyze(mask):
                det = self._evaluate(cand, profile.name, sx, sy, hint)
                if det is not None:
                    results.append(det)

        results.sort(key=lambda d: d.confidence, reverse=True)
        return results

    # ------------------------------------------------------------------
    def _downscale(self, image: np.ndarray) -> np.ndarray:
        h, w = image.shape[:2]
        if w <= self._cfg.work_width:
            return image
        nw = self._cfg.work_width
        nh = max(1, int(round(h * nw / w)))
        return cv2.resize(image, (nw, nh), interpolation=cv2.INTER_AREA)

    def _evaluate(self, c: Candidate, color: str, sx: float, sy: float,
                  hint: Optional[DetectionHint]) -> Optional[PadDetection]:
        cfg = self._cfg

        # --- sert filtreler (sekil) ---
        if c.radius < cfg.min_radius_px:
            return None
        if (c.circularity < cfg.min_circularity or c.solidity < cfg.min_solidity
                or c.axis_ratio < cfg.min_axis_ratio or c.ellipse_iou < cfg.min_ellipse_iou
                or c.rect_fill > cfg.max_rect_fill):
            return None

        # --- islem olcegi -> orijinal cozunurluk ---
        # Piksel-merkezi kuralinda olcek donusumu: u = (u_w + 0.5)/s - 0.5
        u = (c.center[0] + 0.5) / sx - 0.5
        v = (c.center[1] + 0.5) / sy - 0.5
        s_mean = 0.5 * (sx + sy)
        radius = c.radius / s_mean
        area = c.area / (sx * sy)

        # --- boyut ipucu (opsiyonel) ---
        size_score = 1.0
        if hint is not None and hint.expected_radius_px and hint.expected_radius_px > 0.0:
            dev = abs(radius / hint.expected_radius_px - 1.0)
            if dev > cfg.size_tolerance:
                return None
            size_score = 1.0 - 0.5 * dev / cfg.size_tolerance

        # --- guven puani: SIRALAMA icin sezgisel; olasilik degildir ---
        shape = (min(c.circularity, 1.0) + min(c.solidity, 1.0)
                 + min(c.axis_ratio, 1.0) + min(c.ellipse_iou, 1.0)) / 4.0
        conf = shape * size_score * (cfg.border_penalty if c.touches_border else 1.0)
        if conf < cfg.min_confidence:
            return None

        return PadDetection(color=color, center_px=(u, v), radius_px=radius, area_px=area,
                            confidence=conf, circularity=c.circularity,
                            touches_border=c.touches_border)
