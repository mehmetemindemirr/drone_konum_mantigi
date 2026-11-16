"""Test icin sentetik sahne yardimcilari (sadece OpenCV + NumPy)."""
from types import SimpleNamespace

import cv2
import numpy as np

GROUND = (120, 130, 125)      # gri-yesil zemin (BGR)
RED_BGR = (20, 20, 200)       # kirmizi pad
BLUE_BGR = (200, 40, 20)      # mavi pad


def blank(w=640, h=480, bgr=GROUND):
    return np.full((h, w, 3), bgr, np.uint8)


def disc(img, center, r, bgr, squash=1.0, angle=0.0):
    """Dolu daire; squash<1 ise egik bakistan gorunen elips."""
    cv2.ellipse(img, (int(center[0]), int(center[1])),
                (int(r), max(1, int(round(r * squash)))), angle, 0, 360, bgr, -1, cv2.LINE_AA)
    return img


def rect(img, x0, y0, x1, y1, bgr):
    cv2.rectangle(img, (x0, y0), (x1, y1), bgr, -1)
    return img


def polygon(img, pts, bgr):
    cv2.fillPoly(img, [np.array(pts, np.int32)], bgr, cv2.LINE_AA)
    return img


def add_noise(img, sigma, seed=0):
    rng = np.random.default_rng(seed)
    noisy = img.astype(np.float32) + rng.normal(0.0, sigma, img.shape)
    return np.clip(noisy, 0, 255).astype(np.uint8)


def dim(img, k):
    return np.clip(img.astype(np.float32) * k, 0, 255).astype(np.uint8)


def fake_msg(arr, encoding="rgb8", pad_bytes=0):
    """Gercek sensor_msgs/Image'i taklit eder. `arr` KODLAMANIN kanal sirasinda verilir
    (mislabel testi icin bilerek yanlis sirada da verilebilir); satir sonuna dolgu eklenebilir."""
    h, w = arr.shape[:2]
    ch = 1 if arr.ndim == 2 else arr.shape[2]
    rows = arr.reshape(h, w * ch)
    if pad_bytes:
        rows = np.hstack([rows, np.zeros((h, pad_bytes), np.uint8)])
    return SimpleNamespace(height=h, width=w, encoding=encoding,
                           step=rows.shape[1], data=rows.tobytes())


def msg_from_bgr(bgr, encoding="bgr8", pad_bytes=0):
    """BGR goruntuyu istenen kodlamada (dogru etiketle) mesaja cevirir."""
    if encoding == "bgr8":
        arr = bgr
    elif encoding == "rgb8":
        arr = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    elif encoding == "bgra8":
        arr = cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA)
    elif encoding == "rgba8":
        arr = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGBA)
    elif encoding == "mono8":
        arr = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    else:
        raise ValueError(encoding)
    return fake_msg(np.ascontiguousarray(arr), encoding, pad_bytes)
