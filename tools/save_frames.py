#!/usr/bin/env python3
"""Kamera topic'inden kare kaydeder (PNG) + her karenin yanina drone konumunu yazar (CSV).

Bu bir ADAPTOR: ROS'a bagli tek dosya budur. pad_vision paketi ROS'u bilmez.
cv_bridge gerektirmez (goruntu NumPy ile cozulur).

Kullanim (ROS 2 ortami yuklu terminalde):
  python3 tools/save_frames.py --out kareler --every 1.0 --max 120

Uyari: goruntu damgasi Gazebo SIMULASYON saatidir, MAVROS damgasi sistem saatidir.
Bu yuzden CSV'ye iki zaman yazilir: wall_t (bu bilgisayarin saati) ve stamp_s (goruntu damgasi).
"""
import argparse
import csv
import math
import time
from pathlib import Path

import cv2
import numpy as np


def image_msg_to_bgr(msg) -> np.ndarray:
    """sensor_msgs/Image benzeri nesne -> BGR uint8 (H, W, 3)."""
    channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding)
    if channels is None:
        raise ValueError(f"desteklenmeyen goruntu kodlamasi: {msg.encoding}")
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    rows = buf.reshape(msg.height, msg.step)[:, : msg.width * channels]   # satir dolgusunu at
    img = np.ascontiguousarray(rows.reshape(msg.height, msg.width, channels))
    if msg.encoding == "rgb8":
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if msg.encoding == "mono8":
        return cv2.cvtColor(img[:, :, 0], cv2.COLOR_GRAY2BGR)
    return img


def quaternion_to_yaw_deg(x, y, z, w) -> float:
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def run(args):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class FrameSaver(Node):
        def __init__(self):
            super().__init__("frame_saver")
            self.out = Path(args.out)
            self.out.mkdir(parents=True, exist_ok=True)
            self.csv_path = self.out / "kareler.csv"
            self.csv_file = open(self.csv_path, "w", newline="")
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow(["dosya", "wall_t", "stamp_s", "x", "y", "z",
                                  "yaw_enu_deg", "poz_yasi_s"])
            self.pose = None           # (x, y, z, yaw_deg, wall_time)
            self.count = 0
            self.last_save = 0.0
            self.done = False
            # BEST_EFFORT abone, hem RELIABLE hem BEST_EFFORT yayinciya baglanir.
            self.create_subscription(Image, args.image_topic, self.image_cb, qos_profile_sensor_data)
            self.create_subscription(PoseStamped, args.pose_topic, self.pose_cb, qos_profile_sensor_data)
            self.get_logger().info(f"{args.image_topic} dinleniyor, {args.every} sn'de bir kare, "
                                   f"en fazla {args.max}; cikti: {self.out}")

        def pose_cb(self, m):
            p, o = m.pose.position, m.pose.orientation
            self.pose = (p.x, p.y, p.z, quaternion_to_yaw_deg(o.x, o.y, o.z, o.w), time.time())

        def image_cb(self, m):
            now = time.time()
            if self.done or now - self.last_save < args.every:
                return
            try:
                frame = image_msg_to_bgr(m)
            except ValueError as e:
                self.get_logger().error(str(e))
                self.done = True
                return
            name = f"kare_{self.count:04d}.png"
            cv2.imwrite(str(self.out / name), frame)
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if self.pose:
                x, y, z, yaw, t = self.pose
                row = [name, f"{now:.3f}", f"{stamp:.3f}", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}",
                       f"{yaw:.1f}", f"{now - t:.2f}"]
            else:
                row = [name, f"{now:.3f}", f"{stamp:.3f}", "", "", "", "", ""]
            self.writer.writerow(row)
            self.csv_file.flush()
            self.last_save = now
            self.count += 1
            self.get_logger().info(f"{name} kaydedildi (z={row[5] or '-'})")
            if self.count >= args.max:
                self.done = True

    rclpy.init()
    node = FrameSaver()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    print(f"{node.count} kare kaydedildi -> {node.out}  (konumlar: {node.csv_path})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--out", default="kareler")
    ap.add_argument("--every", type=float, default=1.0, help="iki kayit arasi sure (sn)")
    ap.add_argument("--max", type=int, default=120)
    ap.add_argument("--image-topic", default="/drone/down_camera/image_raw")
    ap.add_argument("--pose-topic", default="/mavros/local_position/pose")
    run(ap.parse_args())


if __name__ == "__main__":
    main()
