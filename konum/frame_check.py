#!/usr/bin/env python3
"""
frame_check.py - yerel (x, y, z) ile enlem/boylam AYNI konumu mu anlatiyor? (SALT-OKUNUR)

Bu script drone'a HICBIR komut gondermez, arm etmez, mod degistirmez.
Sadece MAVROS'tan gelen konum verilerini dinler ve karsilastirir.
MAVROS'u kendisi BASLATMAZ: yaw.py (veya elle baslattigin MAVROS) calisiyor olmali.

Ne yapar?
  1) Drone yerdeyken ilk ~2 sn'de "yerel x,y" ile "GPS lat,lon" ciftlerini toplar
     ve yerel orijinin enlem/boylamini (lat0, lon0) hesaplar.
  2) Sonra her saniye:
       - yerel (x, y, z) ve GPS (lat, lon) yazdirir
       - yerel x,y'yi lat/lon'a cevirip gercek GPS ile karsilastirir (metre cinsinden hata)
       - yaw kontrolu: MAVROS yerel yaw (ENU) ile pusula yonunu karsilastirir
  3) Istersen --csv ile her saniyeyi dosyaya kaydeder.

Kullanim:
  python3 frame_check.py
  python3 frame_check.py --csv kayit.csv
"""
import argparse
import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:
    class ExternalShutdownException(Exception):
        pass

CAL_SAMPLES = 10      # kalibrasyon icin kac ornek (0.2 sn aralikla)
STALE_S = 2.0         # veri bu kadar sureden eskiyse "yok" say

COMPASS_ADLARI = ['Kuzey', 'Kuzeydogu', 'Dogu', 'Guneydogu',
                  'Guney', 'Guneybati', 'Bati', 'Kuzeybati']


def m_per_deg(lat_deg):
    """Bu enlemde 1 derece enlem ve 1 derece boylam kac metre (WGS84 serisi)."""
    p = math.radians(lat_deg)
    m_lat = (111132.92 - 559.82 * math.cos(2 * p)
             + 1.175 * math.cos(4 * p) - 0.0023 * math.cos(6 * p))
    m_lon = (111412.84 * math.cos(p) - 93.5 * math.cos(3 * p)
             + 0.118 * math.cos(5 * p))
    return m_lat, m_lon


def local_to_global(x, y, lat0, lon0):
    """Yerel (x=dogu, y=kuzey) metre -> (enlem, boylam)."""
    m_lat, m_lon = m_per_deg(lat0)
    return lat0 + y / m_lat, lon0 + x / m_lon


def global_to_local(lat, lon, lat0, lon0):
    """(enlem, boylam) -> yerel (x=dogu, y=kuzey) metre."""
    m_lat, m_lon = m_per_deg(lat0)
    return (lon - lon0) * m_lon, (lat - lat0) * m_lat


def quaternion_to_yaw_derece(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def compass_adi(derece):
    return COMPASS_ADLARI[int((derece % 360.0 + 22.5) // 45) % 8]


class FrameCheck(Node):
    def __init__(self, csv_path=None):
        super().__init__('frame_check')
        self.t0 = time.time()

        # yerel konum (ENU): x=dogu, y=kuzey, z=yukari
        self.x = self.y = self.z = 0.0
        self.yaw_enu = 0.0
        self.pose_t = 0.0

        # global konum
        self.lat = None
        self.lon = None
        self.fix_t = 0.0

        self.rel_alt = None
        self.compass = None

        self.origin = None   # (lat0, lon0)
        self.cal = []

        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.pose_cb, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 self.fix_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 self.rel_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg',
                                 self.hdg_cb, qos_profile_sensor_data)

        self.create_timer(0.2, self.sample)
        self.create_timer(1.0, self.report)

        self.csv_file = None
        self.csv_writer = None
        if csv_path:
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ['t_s', 'x', 'y', 'z', 'rel_alt', 'lat', 'lon',
                 'yaw_enu_deg', 'compass_deg', 'hata_m'])
            print(f'[KAYIT] CSV: {csv_path}')

    # ---------------- callback'ler ----------------
    def pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        o = msg.pose.orientation
        self.yaw_enu = quaternion_to_yaw_derece(o.x, o.y, o.z, o.w)
        self.pose_t = time.time()

    def fix_cb(self, msg):
        if msg.status.status < 0:
            return  # fix yok
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.fix_t = time.time()

    def rel_cb(self, msg):
        self.rel_alt = msg.data

    def hdg_cb(self, msg):
        self.compass = msg.data

    # ---------------- kalibrasyon ----------------
    def sample(self):
        if self.origin is not None:
            return
        now = time.time()
        if (self.lat is None or now - self.pose_t > STALE_S
                or now - self.fix_t > STALE_S):
            return
        m_lat, m_lon = m_per_deg(self.lat)
        self.cal.append((self.lat - self.y / m_lat, self.lon - self.x / m_lon))
        if len(self.cal) >= CAL_SAMPLES:
            lat0 = sum(c[0] for c in self.cal) / len(self.cal)
            lon0 = sum(c[1] for c in self.cal) / len(self.cal)
            self.origin = (lat0, lon0)
            print('\n' + '=' * 64)
            print('KALIBRASYON TAMAM: yerel orijin (x=0, y=0) su enlem/boylamda:')
            print(f'   lat0 = {lat0:.7f}    lon0 = {lon0:.7f}')
            if abs(self.z) > 1.0:
                print(f'   UYARI: drone havada gorunuyor (z={self.z:.1f} m). '
                      'Hareket varken kalibrasyon biraz hatali cikabilir.')
            print('=' * 64)

    # ---------------- rapor ----------------
    def report(self):
        now = time.time()
        el = now - self.t0
        pose_ok = (now - self.pose_t) < STALE_S
        fix_ok = self.lat is not None and (now - self.fix_t) < STALE_S

        if not (pose_ok and fix_ok):
            print(f'[{el:6.1f}s] veri bekleniyor...  '
                  f'yerel pose={"var" if pose_ok else "YOK"}  '
                  f'GPS={"var" if fix_ok else "YOK"}')
            if el > 10.0:
                print('   ipucu: MAVROS calisiyor mu?')
                print('     ros2 topic hz /mavros/local_position/pose')
                print('     ros2 topic hz /mavros/global_position/global')
            return

        if self.origin is None:
            print(f'[{el:6.1f}s] kalibrasyon: {len(self.cal)}/{CAL_SAMPLES} '
                  'ornek (drone yerde duruyor olmali)')
            return

        lat0, lon0 = self.origin
        ex, ey = global_to_local(self.lat, self.lon, lat0, lon0)
        dx = self.x - ex
        dy = self.y - ey
        hata = math.hypot(dx, dy)

        rel = f'{self.rel_alt:6.2f}' if self.rel_alt is not None else '  yok '
        print('-' * 64)
        print(f'[{el:6.1f}s]')
        print(f'  YEREL : x={self.x:+8.2f} (dogu)  y={self.y:+8.2f} (kuzey)  '
              f'z={self.z:6.2f}  | rel_alt={rel} m')
        print(f'  GLOBAL: lat={self.lat:.7f}  lon={self.lon:.7f}')
        print(f'  Yerel konumu GPS ile karsilastir -> hata = {hata:.2f} m '
              f'(dx={dx:+.2f}, dy={dy:+.2f})')

        if self.compass is not None:
            beklenen = (90.0 - self.yaw_enu) % 360.0
            fark = (self.compass - beklenen + 180.0) % 360.0 - 180.0
            print(f'  YAW   : ENU yaw={self.yaw_enu:+7.1f}   pusula={self.compass:6.1f} '
                  f'({compass_adi(self.compass)})   '
                  f'beklenen(90-yaw)={beklenen:6.1f}   fark={fark:+.1f}')
        else:
            print(f'  YAW   : ENU yaw={self.yaw_enu:+7.1f}   pusula: veri yok')

        if self.csv_writer:
            self.csv_writer.writerow([
                f'{el:.1f}', f'{self.x:.3f}', f'{self.y:.3f}', f'{self.z:.3f}',
                '' if self.rel_alt is None else f'{self.rel_alt:.3f}',
                f'{self.lat:.8f}', f'{self.lon:.8f}', f'{self.yaw_enu:.2f}',
                '' if self.compass is None else f'{self.compass:.2f}',
                f'{hata:.3f}'])
            self.csv_file.flush()

    def close(self):
        if self.csv_file:
            self.csv_file.close()


def main():
    ap = argparse.ArgumentParser(description='Yerel x,y,z ile enlem/boylam karsilastirma (salt-okunur)')
    ap.add_argument('--csv', default=None, help='her saniyeyi bu CSV dosyasina kaydet')
    args = ap.parse_args()

    rclpy.init()
    node = FrameCheck(args.csv)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
