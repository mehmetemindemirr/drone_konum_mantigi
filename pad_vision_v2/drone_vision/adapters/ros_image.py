"""ROS goruntu mesajlari <-> NumPy.

image_msg_to_bgr ROS gerektirmez (mesaj "ordek tipleme" ile okunur: height, width,
encoding, step, data). Yalniz bgr_to_image_msg ve subscribe_image ROS'u ice aktarir,
o da cagrildiklari anda (tembel), boylece bu dosya ROS'suz ortamda da yuklenebilir.
"""
from __future__ import annotations

import cv2
import numpy as np

_CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


def image_msg_to_bgr(msg) -> np.ndarray:
    """sensor_msgs/Image benzeri nesne -> BGR uint8 (H, W, 3). cv_bridge gerekmez."""
    channels = _CHANNELS.get(msg.encoding)
    if channels is None:
        raise ValueError(
            f"desteklenmeyen goruntu kodlamasi: {msg.encoding!r} "
            f"(desteklenen: {', '.join(_CHANNELS)}). camera_ros icin 'format' parametresine bak.")
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.step < msg.width * channels or buf.size != msg.height * msg.step:
        raise ValueError(f"beklenmeyen goruntu verisi: {buf.size} bayt, "
                         f"{msg.height} satir x step {msg.step} bekleniyordu")
    rows = buf.reshape(msg.height, msg.step)[:, : msg.width * channels]   # satir dolgusunu at
    img = np.ascontiguousarray(rows.reshape(msg.height, msg.width, channels))

    enc = msg.encoding
    if enc == "bgr8":
        return img
    if enc == "rgb8":
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return cv2.cvtColor(img[:, :, 0], cv2.COLOR_GRAY2BGR)                 # mono8


def bgr_to_image_msg(frame_bgr: np.ndarray, stamp=None, frame_id: str = "pad_vision"):
    """BGR uint8 dizisi -> sensor_msgs/Image (bgr8). ROS gerektirir (tembel ice aktarim)."""
    from sensor_msgs.msg import Image

    h, w = frame_bgr.shape[:2]
    msg = Image()
    msg.height, msg.width = h, w
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = w * 3
    msg.data = np.ascontiguousarray(frame_bgr).tobytes()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    return msg


def subscribe_image(node, topic: str, callback):
    """Goruntu topic'ine abone ol. BEST_EFFORT abone, hem RELIABLE hem BEST_EFFORT
    yayinciya baglanir; yavas tuketici eski kareleri dusurur, kuyruk sismez."""
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    return node.create_subscription(Image, topic, callback, qos_profile_sensor_data)
