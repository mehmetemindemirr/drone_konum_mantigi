"""Veri tipleri ve ayarlar.

Bu dosya sadece standart kutuphane kullanir (ROS, OpenCV, NumPy YOK).
Tespit sonucunun "sozlesmesi" burada tanimlidir: ana gorev bu tiplere bagimlidir,
dedektorun icinde nasil calistigina degil.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

HsvRange = Tuple[Tuple[int, int, int], Tuple[int, int, int]]  # (alt, ust)


@dataclass(frozen=True)
class ColorProfile:
    """Bir pad rengi = bir veya birden cok HSV araligi (VEYA ile birlesir)."""
    name: str
    ranges: Tuple[HsvRange, ...]


# OpenCV HSV: H 0..179, S 0..255, V 0..255.
# Kirmizi, H ekseninin iki ucunda durur (0 civari ve 179 civari): iki aralik gerekir.
RED = ColorProfile("kirmizi", (
    ((0, 120, 70), (10, 255, 255)),
    ((170, 120, 70), (179, 255, 255)),
))
BLUE = ColorProfile("mavi", (
    ((100, 120, 50), (130, 255, 255)),
))


@dataclass(frozen=True)
class DetectorConfig:
    """Tum esikler tek yerde. Gercek veriyle ayarlanacak degerler burada."""
    work_width: int = 320            # Islem genisligi; daha genis goruntu kucultulur (hiz)
    morph_kernel_frac: float = 0.01  # Morfoloji cekirdegi = oran x islem genisligi (en az 3)
    min_area_px: float = 30.0        # Islem olceginde en kucuk alan (piksel)
    min_radius_px: float = 5.0       # Islem olceginde en kucuk esdeger yaricap
    min_circularity: float = 0.75    # 4*pi*A/P^2  (kusursuz daire ~1)
    min_solidity: float = 0.92       # alan / dis bukey zarf alani
    min_axis_ratio: float = 0.55     # elips kucuk/buyuk eksen (egim toleransi)
    min_ellipse_iou: float = 0.92    # blob ile kendi elipsinin ortusme orani
    max_rect_fill: float = 0.93      # alan / sarmalayan dikdortgen: daire/elips 0.79-0.88, kare 1.0
    size_tolerance: float = 0.5      # boyut ipucu varsa: |r/r_beklenen - 1| siniri
    border_penalty: float = 0.6      # kareye tasan aday icin guven carpani
    min_confidence: float = 0.4      # bunun altindaki adaylar elenir
    max_candidates: int = 10         # renk basina incelenecek en buyuk blob sayisi


@dataclass(frozen=True)
class DetectionHint:
    """Cagiranin dedektore verdigi istege bagli ipucu.

    expected_radius_px: pad'in ORIJINAL goruntu cozunurlugunde beklenen yaricapi.
    Irtifa ve kamera bilindiginde expected_radius_px() ile hesaplanir.
    """
    expected_radius_px: Optional[float] = None


@dataclass(frozen=True)
class PadDetection:
    color: str                        # "kirmizi" / "mavi"
    center_px: Tuple[float, float]    # (u, v), ORIJINAL goruntu pikseli, piksel merkezi = tam sayi
    radius_px: float                  # esdeger yaricap, orijinal cozunurlukte
    area_px: float                    # orijinal cozunurlukte alan
    confidence: float                 # 0..1 SIRALAMA puani (olasilik degil, veriyle kalibre edilecek)
    circularity: float
    touches_border: bool              # True ise pad kareden tasiyor: merkez ONYARGILI olabilir


def expected_radius_px(altitude_m: float, fx_px: float, pad_radius_m: float = 1.0) -> float:
    """Dik bakan kamerada pad'in goruntudeki yaricapi: r = fx * R / h.

    altitude_m: kameranin pad duzlemine dikey mesafesi.
    fx_px: odak uzakligi, KULLANILAN goruntu cozunurlugunde piksel cinsinden.
    """
    if altitude_m <= 0.0:
        raise ValueError("altitude_m > 0 olmali")
    return fx_px * pad_radius_m / altitude_m
