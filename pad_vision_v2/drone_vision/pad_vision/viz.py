"""Hata ayiklama icin goruntu uzerine tespit cizimi. Tespit mantigina bagimli degildir."""
from __future__ import annotations

from typing import Iterable

import cv2
import numpy as np

from .models import PadDetection

_GREEN = (0, 255, 0)


def draw_detections(image_bgr: np.ndarray, detections: Iterable[PadDetection]) -> np.ndarray:
    """Girdiyi degistirmez; cizimli bir kopya dondurur (yalnizca ASCII etiket)."""
    out = image_bgr.copy()
    for d in detections:
        u, v = int(round(d.center_px[0])), int(round(d.center_px[1]))
        cv2.circle(out, (u, v), max(1, int(round(d.radius_px))), _GREEN, 2, cv2.LINE_AA)
        cv2.drawMarker(out, (u, v), _GREEN, cv2.MARKER_CROSS, 14, 2)
        label = f"{d.color} {d.confidence:.2f}" + (" KENAR" if d.touches_border else "")
        cv2.putText(out, label, (max(0, u - 40), max(14, v - int(d.radius_px) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, _GREEN, 1, cv2.LINE_AA)
    return out
