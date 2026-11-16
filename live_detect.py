#!/usr/bin/env python3
"""CANLI pad gozlemcisi: ayri surec, ucusa dokunmaz.

Kamera topic'ine + MAVROS konumuna abone olur, PadWatcher ile tespit eder, konsola yazar,
istersen isaretli goruntuyu bir topic'e yayinlar (rqt_image_view ile izle) veya pencerede gosterir.

  # simulasyon (yaw.py baska terminalde ucarken):
  python3 tools/live_detect.py --topic /drone/down_camera/image_raw --hfov 80 --use-hint

  # gercek kamera (Pi), HSV rengi kaymis olabilir -> once kaplama/tespit satirlarina bak:
  python3 tools/live_detect.py --topic /camera/image_raw --hfov 62.2 --work-width 320

Hicbir karar vermez; konum kaydetme / inis sonraki moduller.
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from adapters.pad_watcher import PadWatcher  # noqa: E402
from adapters.ros_image import bgr_to_image_msg, subscribe_image  # noqa: E402
from pad_vision import ColorCoverage, DetectorConfig, HsvPadDetector, draw_detections  # noqa: E402


def build_watcher(args, log=lambda s: print(s, flush=True)) -> PadWatcher:
    """ROS'suz: argumanlardan PadWatcher kurar (test edilebilir)."""
    cfg = DetectorConfig(work_width=args.work_width)
    return PadWatcher(HsvPadDetector(config=cfg), ColorCoverage(),
                      hfov_deg=args.hfov, use_hint=args.use_hint,
                      every_s=args.every, swap_rb=args.swap_rb, log=log)


def run(args):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    watcher = build_watcher(args)

    class Live(Node):
        def __init__(self):
            super().__init__("pad_live")
            self.alt = None
            subscribe_image(self, args.topic, watcher.on_image)
            self.create_subscription(PoseStamped, args.pose_topic, self.pose_cb, qos_profile_sensor_data)
            self.pub = self.create_publisher(Image, "/pad_vision/debug_image", 1) if args.publish else None
            self.create_timer(0.1, self.tick)
            self.get_logger().info(f"{args.topic} dinleniyor (hfov={args.hfov}, ipucu={args.use_hint})")

        def pose_cb(self, m):
            self.alt = m.pose.position.z

        def tick(self):
            before = watcher.last_report
            rep = watcher.update(self.alt)
            if rep is None or rep is before:
                return                                   # yeni isleme yok
            if self.pub or args.show:
                import cv2
                img = draw_detections(watcher.last_frame.copy(), rep.detections)
                if self.pub:
                    self.pub.publish(bgr_to_image_msg(img))
                if args.show:
                    cv2.imshow("pad", img)
                    cv2.waitKey(1)

    rclpy.init()
    node = Live()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--topic", default="/drone/down_camera/image_raw")
    ap.add_argument("--pose-topic", default="/mavros/local_position/pose")
    ap.add_argument("--hfov", type=float, default=None, help="yatay gorus acisi (derece)")
    ap.add_argument("--use-hint", action="store_true", help="irtifadan beklenen yaricap ipucu (--hfov gerekir)")
    ap.add_argument("--work-width", type=int, default=320, help="tespit icin kucultme genisligi")
    ap.add_argument("--every", type=float, default=0.5, help="islem periyodu (sn)")
    ap.add_argument("--swap-rb", action="store_true", help="kirmizi/mavi ters gorunuyorsa kanal sirasini cevir")
    ap.add_argument("--publish", action="store_true", help="/pad_vision/debug_image yayinla")
    ap.add_argument("--show", action="store_true", help="OpenCV penceresi (masaustu gerekir)")
    args = ap.parse_args(argv)
    if args.use_hint and not args.hfov:
        ap.error("--use-hint icin --hfov gerekli")
    return args


if __name__ == "__main__":
    run(parse())
