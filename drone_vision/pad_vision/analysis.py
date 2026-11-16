"""Sekil analizi: ikili maske -> aday blob'lar ve olculeri. Tek sorumluluk.

Karar VERMEZ (hangi aday pad'dir sorusu dedektorun isi); sadece olcer.
Koordinatlar maskenin kendi piksel olcegindedir, piksel merkezi = tam sayi.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class Candidate:
    center: Tuple[float, float]   # (u, v)
    radius: float                 # esdeger yaricap = sqrt(alan / pi)
    area: float                   # piksel sayisi (tam)
    circularity: float            # 4*pi*A/P^2
    solidity: float               # alan / dis bukey zarf alani
    axis_ratio: float             # fitEllipse kucuk/buyuk eksen
    ellipse_iou: float            # blob ile kendi elipsinin kesisim/birlesim orani
    rect_fill: float              # alan / en kucuk sarmalayan dikdortgen alani
                                  #   elips (her egimde) ~0.785, kare/dikdortgen ~1.0
    touches_border: bool


class ShapeAnalyzer:
    def __init__(self, min_area_px: float = 30.0, max_candidates: int = 10):
        self._min_area = min_area_px
        self._max_candidates = max_candidates

    def analyze(self, mask: np.ndarray) -> List[Candidate]:
        # OpenCV 3 -> 3 deger, 4/5 -> 2 deger dondurur; [-2] ikisinde de konturlar.
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        img_h, img_w = mask.shape[:2]

        out: List[Candidate] = []
        for cnt in contours:
            if len(cnt) < 5:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)

            # Kontur ici dolu maske: alan ve agirlik merkezi PIKSEL-TAM olur.
            roi = np.zeros((bh, bw), np.uint8)
            shifted = (cnt - np.array([x, y])).astype(np.int32)
            cv2.drawContours(roi, [shifted], -1, 255, -1)
            area = float(cv2.countNonZero(roi))
            if area < self._min_area:
                continue
            m = cv2.moments(roi, binaryImage=True)
            cx = m["m10"] / m["m00"] + x
            cy = m["m01"] / m["m00"] + y

            perimeter = cv2.arcLength(cnt, True)
            poly_area = cv2.contourArea(cnt)
            if perimeter <= 0.0 or poly_area <= 0.0:
                continue
            circularity = 4.0 * math.pi * poly_area / (perimeter * perimeter)

            hull_area = cv2.contourArea(cv2.convexHull(cnt))
            solidity = poly_area / hull_area if hull_area > 0.0 else 0.0

            try:
                (ecx, ecy), (d1, d2), ang = cv2.fitEllipse(cnt)
            except cv2.error:
                continue
            if max(d1, d2) <= 0.0:
                continue
            axis_ratio = min(d1, d2) / max(d1, d2)

            ell = np.zeros((bh, bw), np.uint8)
            cv2.ellipse(ell, ((ecx - x, ecy - y), (d1, d2), ang), 255, -1)
            inter = float(cv2.countNonZero(cv2.bitwise_and(roi, ell)))
            union = float(cv2.countNonZero(cv2.bitwise_or(roi, ell)))
            ellipse_iou = inter / union if union > 0.0 else 0.0

            # Poligon alani / dikdortgen alani: ikisi de ayni "piksel-merkezi" cercevesinde
            # olculur, boylece kucuk disklerde de oran ~0.785 kalir.
            (_, (rw, rh), _) = cv2.minAreaRect(cnt)
            rect_fill = poly_area / (rw * rh) if rw * rh > 0.0 else 1.0

            touches = x <= 0 or y <= 0 or (x + bw) >= img_w or (y + bh) >= img_h

            out.append(Candidate((cx, cy), math.sqrt(area / math.pi), area,
                                 circularity, solidity, axis_ratio, ellipse_iou,
                                 rect_fill, touches))

        out.sort(key=lambda c: c.area, reverse=True)
        return out[: self._max_candidates]
