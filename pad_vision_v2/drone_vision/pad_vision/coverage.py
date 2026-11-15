"""Kadraj kaplama olcusu: goruntunun ne kadari bir pad rengiyle kapli (0..1).

Neden ayri bir bilesen?
  Pad kadraja sigmayacak kadar yaklasinca (son ~1-2 m) daire SEKLI kaybolur;
  dedektor bunu bilerek reddeder (merkezi guvenilir cikarilamaz). Ama "kadrajin
  neredeyse tamami kirmizi" bilgisi hala degerlidir: drone pad'in tam ustundedir.
  Bu sinyali dairesel sekil aramadan, ucuza verir.
"""
from __future__ import annotations

from typing import Dict, Optional, Sequence

import cv2
import numpy as np

from .imaging import downscale_to_width, ensure_bgr
from .models import BLUE, RED, ColorProfile
from .segmentation import ColorSegmenter


class ColorCoverage:
    def __init__(self, profiles: Sequence[ColorProfile] = (RED, BLUE),
                 work_width: int = 160,
                 segmenter: Optional[ColorSegmenter] = None):
        self._profiles = tuple(profiles)
        self._work_width = work_width
        self._segmenter = segmenter or ColorSegmenter(0.01)

    def measure(self, image_bgr: np.ndarray) -> Dict[str, float]:
        """{renk_adi: kadrajin o renkle kapli orani (0..1)}"""
        ensure_bgr(image_bgr)
        work = downscale_to_width(image_bgr, self._work_width)
        hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
        total = float(hsv.shape[0] * hsv.shape[1])
        return {p.name: cv2.countNonZero(self._segmenter.mask(hsv, p)) / total
                for p in self._profiles}
