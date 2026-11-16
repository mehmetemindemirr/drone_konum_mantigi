#!/usr/bin/env python3
"""Klasordeki (veya tek) goruntulerde pad tespiti calistirir, isaretli kopyalari kaydeder.

ROS gerektirmez. Kare kaydedicinin (save_frames.py) urettigi klasorle calisir.

Kullanim:
  python3 tools/detect_images.py kareler/ --out sonuc/
  python3 tools/detect_images.py kareler/ --out sonuc/ --poses kareler/kareler.csv --fx 381.4
  python3 tools/detect_images.py kare.png --expected-radius 25

Boyut ipucu (yanlis pozitifleri azaltir), iki yoldan biriyle verilir:
  --expected-radius R     tum karelerde ayni beklenen yaricap (piksel)
  --poses CSV --fx FX     her karenin irtifasi (CSV'deki z) ile r = fx*1/z hesaplanir
"""
import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2  # noqa: E402

from pad_vision import (DetectionHint, DetectorConfig, HsvPadDetector,  # noqa: E402
                        draw_detections, expected_radius_px)

IMG_EXT = {".png", ".jpg", ".jpeg", ".bmp"}
MIN_ALT_FOR_HINT = 1.0   # bundan alcaktaki karelerde ipucu verme (pad kadrajdan tasar)


def collect(path: Path):
    if path.is_dir():
        return sorted(p for p in path.iterdir() if p.suffix.lower() in IMG_EXT)
    return [path]


def load_altitudes(csv_path: Path):
    alts = {}
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                alts[row["dosya"]] = float(row["z"])
            except (KeyError, ValueError):
                continue
    return alts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("girdi", type=Path, help="goruntu dosyasi veya klasor")
    ap.add_argument("--out", type=Path, default=None, help="isaretli kopyalarin yazilacagi klasor")
    ap.add_argument("--work-width", type=int, default=DetectorConfig().work_width)
    ap.add_argument("--expected-radius", type=float, default=None)
    ap.add_argument("--poses", type=Path, default=None, help="save_frames.py'nin urettigi CSV")
    ap.add_argument("--fx", type=float, default=None, help="odak uzakligi (px), --poses ile birlikte")
    args = ap.parse_args()

    files = collect(args.girdi)
    if not files:
        print("goruntu bulunamadi:", args.girdi)
        return 1
    if args.poses and not args.fx:
        ap.error("--poses icin --fx de gerekli")

    altitudes = load_altitudes(args.poses) if args.poses else {}
    detector = HsvPadDetector(config=DetectorConfig(work_width=args.work_width))
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)

    n_hit = 0
    for path in files:
        img = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if img is None:
            print(f"{path.name}: okunamadi, atlandi")
            continue

        hint = None
        if args.expected_radius:
            hint = DetectionHint(args.expected_radius)
        elif altitudes.get(path.name, 0.0) >= MIN_ALT_FOR_HINT:
            hint = DetectionHint(expected_radius_px(altitudes[path.name], args.fx))

        dets = detector.detect(img, hint)
        n_hit += bool(dets)
        hint_txt = f" [ipucu r~{hint.expected_radius_px:.0f}px]" if hint else ""
        print(f"{path.name}: {len(dets)} tespit{hint_txt}")
        for d in dets:
            print(f"    {d.color:8s} u={d.center_px[0]:7.1f} v={d.center_px[1]:7.1f} "
                  f"r={d.radius_px:6.1f}px guven={d.confidence:.2f}"
                  f"{'  KENARDA' if d.touches_border else ''}")
        if args.out:
            cv2.imwrite(str(args.out / path.name), draw_detections(img, dets))

    print(f"\nOzet: {n_hit}/{len(files)} karede tespit var.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
