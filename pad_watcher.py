"""PadWatcher: ana goreve takilan GOZLEMCI.

Karar VERMEZ, ucusa MUDAHALE ETMEZ: sadece en son kareyi alir, dedektoru calistirir,
gorulen/kaybolan pad'i ve kadraj kaplamasini konsola yazar.

  - on_image(msg)      : abonelik callback'i; yalniz en son mesaji saklar (cok ucuz)
  - update(altitude_m) : ana gorev ne zaman isterse cagirir; en cok every_s'de bir isler

Bu dosya ROS'u ice aktarmaz: mesaj nesnesini ve saati disaridan alir, bu yuzden ROS'suz test edilir.
Gozlemci ucusu ASLA bozmamali: update() icinde olusan her hata yakalanir ve yazilir.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Set, Tuple

import numpy as np

from pad_vision import (ColorCoverage, DetectionHint, PadDetection, PadDetector,
                        expected_radius_px)

from .ros_image import image_msg_to_bgr


@dataclass(frozen=True)
class WatchReport:
    time: float                          # clock() zamani
    frame_size: Tuple[int, int]          # (genislik, yukseklik)
    altitude_m: Optional[float]
    detections: Tuple[PadDetection, ...]  # guvene gore azalan
    coverage: Dict[str, float]           # renk -> kadrajin kapli orani (0..1)


class PadWatcher:
    def __init__(self,
                 detector: PadDetector,
                 coverage: Optional[ColorCoverage] = None,
                 hfov_deg: Optional[float] = None,
                 use_hint: bool = False,
                 every_s: float = 0.5,
                 status_every_s: float = 2.0,
                 lost_after: int = 3,
                 cover_alert: float = 0.8,
                 max_frame_age_s: float = 2.0,
                 min_alt_for_hint_m: float = 1.0,
                 swap_rb: bool = False,
                 pad_radius_m: float = 1.0,
                 log: Callable[[str], None] = lambda s: print(s, flush=True),
                 clock: Callable[[], float] = time.monotonic):
        if use_hint and not hfov_deg:
            raise ValueError("use_hint=True icin hfov_deg gerekli")
        self._detector = detector
        self._coverage = coverage
        self._hfov_deg = hfov_deg
        self._use_hint = use_hint
        self._every_s = every_s
        self._status_every = status_every_s
        self._lost_after = lost_after
        self._cover_alert = cover_alert
        self._max_age = max_frame_age_s
        self._min_alt_hint = min_alt_for_hint_m
        self._swap_rb = swap_rb
        self._pad_radius_m = pad_radius_m
        self._log = log
        self._clock = clock

        self.last_report: Optional[WatchReport] = None
        self.last_frame: Optional[np.ndarray] = None

        self._latest = None                       # (mesaj, alinma zamani)
        self._last_run = float("-inf")
        self._misses: Dict[str, int] = {}
        self._announced: Set[str] = set()
        self._last_status: Dict[str, float] = {}
        self._covering: Set[str] = set()
        self._warned: Dict[str, float] = {}

    # ---------------- abonelik tarafi ----------------
    def on_image(self, msg) -> None:
        self._latest = (msg, self._clock())

    # ---------------- ana gorev tarafi ----------------
    def update(self, altitude_m: Optional[float] = None) -> Optional[WatchReport]:
        now = self._clock()
        if self._latest is None:
            self._warn("nodata", now, 5.0, "[PAD] henuz goruntu gelmedi (kamera topic'i dogru mu?)")
            return None
        if now - self._last_run < self._every_s:
            return self.last_report
        self._last_run = now

        msg, rx = self._latest
        if now - rx > self._max_age:
            self._warn("stale", now, 5.0, f"[PAD] goruntu {now - rx:.1f} sn'dir guncellenmedi")
            return None

        try:
            frame = image_msg_to_bgr(msg)
            if self._swap_rb:
                frame = np.ascontiguousarray(frame[:, :, ::-1])
            h, w = frame.shape[:2]
            hint = self._make_hint(w, altitude_m)
            dets = tuple(self._detector.detect(frame, hint))
            cov = dict(self._coverage.measure(frame)) if self._coverage else {}
        except Exception as e:  # noqa: BLE001  gozlemci ucusu bozmamali
            self._warn("err:" + type(e).__name__, now, 10.0, f"[PAD] hata ({type(e).__name__}): {e}")
            return None

        report = WatchReport(now, (w, h), altitude_m, dets, cov)
        self.last_report, self.last_frame = report, frame
        self._announce(report, now)
        return report

    # ---------------- ic yardimcilar ----------------
    def _make_hint(self, width: int, altitude_m: Optional[float]) -> Optional[DetectionHint]:
        if not self._use_hint or altitude_m is None or altitude_m < self._min_alt_hint:
            return None
        fx = (width / 2.0) / math.tan(math.radians(self._hfov_deg) / 2.0)
        return DetectionHint(expected_radius_px(altitude_m, fx, self._pad_radius_m))

    def _warn(self, key: str, now: float, period: float, text: str) -> None:
        last = self._warned.get(key)
        if last is None or now - last >= period:
            self._warned[key] = now
            self._log(text)

    def _announce(self, rep: WatchReport, now: float) -> None:
        best: Dict[str, PadDetection] = {}
        for d in rep.detections:                 # guvene gore sirali: ilk gelen en iyi
            best.setdefault(d.color, d)

        for color, d in best.items():
            self._misses[color] = 0
            if color not in self._announced:
                self._announced.add(color)
                self._last_status[color] = now
                self._log(self._line("gorundu", d, rep))
            elif now - self._last_status[color] >= self._status_every:
                self._last_status[color] = now
                self._log(self._line("goruluyor", d, rep))

        for color in list(self._announced - set(best)):
            self._misses[color] = self._misses.get(color, 0) + 1
            if self._misses[color] >= self._lost_after:
                self._announced.discard(color)
                self._misses[color] = 0
                self._log(f"[PAD] {color.upper()} kayboldu")

        for color, frac in rep.coverage.items():
            if frac >= self._cover_alert and color not in self._covering:
                self._covering.add(color)
                self._log(f"[PAD] KADRAJIN %{frac * 100:.0f}'I {color.upper()} -> "
                          f"pad'in tam ustundeyiz (daire sekli artik gorunmuyor)")
            elif frac < 0.7 * self._cover_alert and color in self._covering:
                self._covering.discard(color)
                self._log(f"[PAD] kadraj artik {color} degil")

    def _line(self, kind: str, d: PadDetection, rep: WatchReport) -> str:
        alt = f"{rep.altitude_m:.1f} m" if rep.altitude_m is not None else "?"
        txt = (f"[PAD] {d.color.upper()} {kind}  merkez=({d.center_px[0]:.0f},{d.center_px[1]:.0f}) px  "
               f"r={d.radius_px:.0f} px  guven={d.confidence:.2f}  irtifa={alt}")
        if d.touches_border:
            txt += "  KADRAJ KENARINDA"
        est = self._fx_estimate(d, rep)
        if est is not None:
            fx, hfov = est
            txt += f"  olculen fx~{fx:.0f} px (yatay gorus ~{hfov:.0f} derece)"
        return txt

    def _fx_estimate(self, d: PadDetection, rep: WatchReport) -> Optional[Tuple[float, float]]:
        """Bilinen pad yaricapi + irtifadan odak uzakligini olc: fx = r_px * h / R.

        Gercek kameranin gorus alanini DOGRULAMAK icindir. Yalniz yeterince yuksekte ve
        pad kadraj kenarinda degilken (yaricap hatasi kucukken) hesaplanir.
        """
        if rep.altitude_m is None or rep.altitude_m < 3.0 or d.touches_border:
            return None
        fx = d.radius_px * rep.altitude_m / self._pad_radius_m
        hfov = 2.0 * math.degrees(math.atan((rep.frame_size[0] / 2.0) / fx))
        return fx, hfov
