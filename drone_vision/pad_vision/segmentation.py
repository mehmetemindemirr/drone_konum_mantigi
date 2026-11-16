"""Renk bolutleme: HSV goruntu + renk profili -> ikili maske. Tek sorumluluk."""
from __future__ import annotations

import cv2
import numpy as np

from .models import ColorProfile


class ColorSegmenter:
    def __init__(self, kernel_frac: float = 0.01):
        self._kernel_frac = kernel_frac

    def mask(self, hsv: np.ndarray, profile: ColorProfile) -> np.ndarray:
        h, w = hsv.shape[:2]
        mask = np.zeros((h, w), np.uint8)
        for lo, hi in profile.ranges:
            mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))

        k = max(3, int(round(self._kernel_frac * w)))
        if k % 2 == 0:
            k += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)    # tek tuk benekleri sil
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)   # ic delikleri kapat
        return mask
