#!/usr/bin/env python3
"""
MAVROS (ArduPilot) kalkış - havada bekleme - iniş - disarm testi.

Akış (durum makinesi):
  WAIT_CONNECT -> GUIDED -> ARM -> TAKEOFF -> CLIMB -> HOVER -> LAND -> DONE

Önemli notlar:
  * ArduPilot seri porttan bağlanınca konum/irtifa mesajlarını kendiliğinden
    göndermez. Bağlanınca /mavros/set_stream_rate ile akış istenir.
  * local_position/pose, global_position/rel_alt ve vfr_hud irtifaları
    "yere göre" değil, EKF/home başlangıcına göre ve barometre kaymasına
    açıktır. Bu yüzden ARM anındaki değer sıfır kabul edilir (offset).
  * Yere bakan lidar (rangefinder) geçerli veri veriyorsa en güvenilir
    kaynak odur ve önceliklidir (yere göre doğrudan mesafe).
"""
import math
import subprocess
import time
import os
import signal
import atexit

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy, qos_profile_sensor_data)
from mavros_msgs.msg import State, VfrHud
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Float64

# ----------------------- AYARLAR -----------------------
# FCU_URL = 'udp://127.0.0.1:14550@14555'   # SITL
# FCU_URL = '/dev/ttyACM0:115200'
FCU_URL = '/dev/ttyS0:115200'
TARGET_ALT = 5.0        # metre (kalkış yüksekliği)
HOVER_SECONDS = 10.0    # havada kalma süresi (saniye)
ALT_TOLERANCE = 0.3     # hedef irtifaya bu kadar yaklaşınca "ulaştı" say
CLIMB_TIMEOUT = 30.0    # bu sürede irtifaya çıkamazsa inişe geç
STREAM_RATE_HZ = 10     # istenen telemetri akış hızı
DATA_FRESH_SEC = 1.0    # bu süreden eski veri "bayat" sayılır
USE_LIDAR = True        # geçerli lidar verisi varsa öncelikli kullan
LIDAR_TOPIC = '/mavros/distance_sensor/rangefinder_pub'
GROUND_ALT = 0.3        # LAND'de bu irtifanın altı "yerde" sayılır
# -------------------------------------------------------

BARO_SOURCES = ('pose', 'rel_alt', 'vfr_hud')   # offset gerektiren kaynaklar

mavros_proc = None


def cleanup():
    global mavros_proc
    if mavros_proc and mavros_proc.poll() is None:
        print('\n[CLEANUP] MAVROS kapatılıyor...')
        try:
            os.killpg(os.getpgid(mavros_proc.pid), signal.SIGTERM)
            mavros_proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(mavros_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        print('[CLEANUP] MAVROS kapatıldı.')


def start_mavros():
    global mavros_proc
    print('[BAŞLAT] MAVROS başlatılıyor...')
    cmd = ['ros2', 'launch', 'mavros', 'apm.launch', f'fcu_url:={FCU_URL}']
    mavros_proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    atexit.register(cleanup)
    for i in range(5, 0, -1):
        print(f'\r[BAŞLAT] {i} saniye bekleniyor...', end='', flush=True)
        time.sleep(1)
    print('\n[BAŞLAT] Servisler bekleniyor...\n')


class TakeoffTest(Node):
    def __init__(self):
        super().__init__('mavros_takeoff_test')

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.state = None
        self.phase = 'WAIT_CONNECT'
        self.finished = False
        self.stream_done = False

        # Ham irtifa kaynakları: isim -> [değer, son güncelleme zamanı]
        self.src = {
            'lidar': [None, 0.0],     # yere mesafe (m)
            'pose': [None, 0.0],      # local_position/pose z
            'rel_alt': [None, 0.0],   # global_position/rel_alt
            'vfr_hud': [None, 0.0],   # vfr_hud altitude
        }
        # ARM anında alınan sıfır noktaları (barometrik kaynaklar için)
        self.offsets = {}
        self.offsets_set = False

        self.pending = None      # bekleyen servis çağrısı (future)
        self.pending_tag = None
        self.last_req = 0.0
        self.climb_start = 0.0
        self.hover_start = 0.0
        self.land_start = 0.0

        self.create_subscription(State, '/mavros/state', self.state_cb, state_qos)
        # Sensör verileri BEST_EFFORT yayınlanır, sensor QoS şart
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.pose_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 self.rel_alt_cb, qos_profile_sensor_data)
        self.create_subscription(VfrHud, '/mavros/vfr_hud',
                                 self.vfr_cb, qos_profile_sensor_data)
        self.create_subscription(Range, LIDAR_TOPIC,
                                 self.lidar_cb, qos_profile_sensor_data)

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.stream_client = self.create_client(StreamRate, '/mavros/set_stream_rate')

        for name, cli in (('arming', self.arm_client),
                          ('takeoff', self.takeoff_client),
                          ('set_mode', self.mode_client)):
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{name} servisi bekleniyor...')

        if not self.stream_client.wait_for_service(timeout_sec=3.0):
            print('[UYARI] set_stream_rate servisi yok, akış isteği atlanıyor')
            self.stream_done = True

        self.create_timer(0.2, self.tick)       # durum makinesi (5 Hz)
        self.create_timer(1.0, self.print_status)

    # ---------------- Callback'ler ----------------
    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        self.src['pose'] = [msg.pose.position.z, time.time()]

    def rel_alt_cb(self, msg):
        self.src['rel_alt'] = [msg.data, time.time()]

    def vfr_cb(self, msg):
        self.src['vfr_hud'] = [msg.altitude, time.time()]

    def lidar_cb(self, msg):
        r = msg.range
        if not math.isfinite(r):
            return
        if msg.max_range > 0 and not (msg.min_range <= r <= msg.max_range):
            return  # menzil dışı / geçersiz okuma: güncelleme, bayatlasın
        self.src['lidar'] = [r, time.time()]

    # ---------------- İrtifa seçimi ----------------
    def is_fresh(self, name):
        val, t = self.src[name]
        return val is not None and t > 0 and (time.time() - t) < DATA_FRESH_SEC

    def capture_offsets(self):
        """ARM anında barometrik kaynakların değerini sıfır noktası yap."""
        self.offsets = {}
        for name in BARO_SOURCES:
            self.offsets[name] = self.src[name][0] if self.is_fresh(name) else None
        self.offsets_set = True
        txt = ', '.join(
            f'{n}={self.offsets[n]:.2f}' if self.offsets[n] is not None else f'{n}=yok'
            for n in BARO_SOURCES)
        print(f'[BİLGİ] İrtifa sıfır noktaları alındı: {txt}')

    def get_alt(self):
        """(irtifa, kaynak etiketi, yaş_sn) döndürür. Veri yoksa (None, 'YOK', None).

        Öncelik: lidar > local_pose > rel_alt > vfr_hud
        Barometrik kaynaklar ARM sonrası offset çıkarılarak (yere göre) verilir.
        ARM öncesi ham değer gösterilir ('ham' etiketi).
        """
        now = time.time()
        order = (['lidar'] if USE_LIDAR else []) + list(BARO_SOURCES)
        for name in order:
            if not self.is_fresh(name):
                continue
            val, t = self.src[name]
            age = now - t
            if name == 'lidar':
                return val, 'lidar', age
            if not self.offsets_set:
                return val, f'{name}-ham', age
            off = self.offsets.get(name)
            if off is None:
                continue  # arm anında verisi yoktu, güvenilir değil
            return val - off, name, age
        return None, 'YOK', None

    # ---------------- Yardımcılar ----------------
    def can_send(self, interval):
        """Bekleyen istek yoksa ve son istekten interval saniye geçtiyse True."""
        return self.pending is None and (time.time() - self.last_req) > interval

    def send(self, client, req, tag):
        self.pending = client.call_async(req)
        self.pending_tag = tag
        self.last_req = time.time()

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.send(self.mode_client, req, f'mode:{mode}')

    def set_phase(self, new_phase):
        print(f'\n[FAZ] {self.phase} -> {new_phase}')
        self.phase = new_phase

    def handle_result(self, tag, res):
        if res is None:
            print(f'[HATA] {tag}: cevap alınamadı')
            return
        if tag == 'stream':
            print(f'[SONUÇ] Telemetri akışı {STREAM_RATE_HZ} Hz olarak istendi')
            self.stream_done = True
        elif tag == 'takeoff':
            if res.success:
                print(f'[SONUÇ] Kalkış komutu kabul edildi ({TARGET_ALT} m)')
                self.climb_start = time.time()
                self.set_phase('CLIMB')
            else:
                print('[SONUÇ] Kalkış komutu reddedildi, tekrar denenecek')
        elif tag == 'arm':
            print('[SONUÇ] Arm komutu kabul edildi' if res.success
                  else '[SONUÇ] Arm reddedildi (pre-arm: GPS/EKF hazır mı?), tekrar denenecek')
        elif tag == 'disarm':
            print('[SONUÇ] Disarm kabul edildi' if res.success
                  else '[SONUÇ] Disarm reddedildi')

    # ---------------- Durum makinesi ----------------
    def tick(self):
        # Önce bekleyen servis cevabını işle
        if self.pending is not None:
            if not self.pending.done():
                return
            try:
                res = self.pending.result()
            except Exception as e:
                print(f'[HATA] {self.pending_tag}: {e}')
                res = None
            tag = self.pending_tag
            self.pending = None
            self.handle_result(tag, res)

        if self.state is None or self.finished:
            return

        s = self.state

        if self.phase == 'WAIT_CONNECT':
            if s.connected:
                if not self.stream_done:
                    if self.can_send(2.0):
                        print('[KOMUT] Telemetri akışı isteniyor...')
                        req = StreamRate.Request()
                        req.stream_id = 0           # 0 = tüm akışlar
                        req.message_rate = STREAM_RATE_HZ
                        req.on_off = True
                        self.send(self.stream_client, req, 'stream')
                else:
                    self.set_phase('GUIDED')

        elif self.phase == 'GUIDED':
            if s.mode == 'GUIDED':
                self.set_phase('ARM')
            elif self.can_send(2.0):
                print('[KOMUT] GUIDED moduna geçiliyor...')
                self.set_mode('GUIDED')

        elif self.phase == 'ARM':
            if s.armed:
                self.capture_offsets()      # yerdeyken irtifayı sıfırla
                self.set_phase('TAKEOFF')
            elif self.can_send(3.0):
                print('[KOMUT] Arm isteği gönderiliyor...')
                req = CommandBool.Request()
                req.value = True
                self.send(self.arm_client, req, 'arm')

        elif self.phase == 'TAKEOFF':
            # ArduPilot: arm sonrası ~15 sn içinde kalkış verilmezse tekrar disarm eder
            if not s.armed:
                print('[UYARI] Dron disarm oldu, arm aşamasına dönülüyor')
                self.offsets_set = False
                self.set_phase('ARM')
            elif self.can_send(1.0):
                print(f'[KOMUT] Kalkış: {TARGET_ALT} m')
                req = CommandTOL.Request()
                req.altitude = float(TARGET_ALT)
                self.send(self.takeoff_client, req, 'takeoff')

        elif self.phase == 'CLIMB':
            a, _, _ = self.get_alt()
            if a is not None and a >= TARGET_ALT - ALT_TOLERANCE:
                print(f'[BİLGİ] Hedef irtifaya ulaşıldı ({a:.2f} m), {HOVER_SECONDS:.0f} sn bekleniyor')
                self.hover_start = time.time()
                self.set_phase('HOVER')
            elif time.time() - self.climb_start > CLIMB_TIMEOUT:
                print('[UYARI] İrtifaya çıkma zaman aşımı, iniş başlatılıyor')
                self.set_phase('LAND')

        elif self.phase == 'HOVER':
            if time.time() - self.hover_start >= HOVER_SECONDS:
                print('[BİLGİ] Bekleme bitti, iniş başlıyor')
                self.land_start = time.time()
                self.set_phase('LAND')

        elif self.phase == 'LAND':
            if not self.land_start:
                self.land_start = time.time()
            a, _, _ = self.get_alt()
            if not s.armed:
                print('[SONUÇ] DRON İNDİ VE DISARM OLDU!')
                self.set_phase('DONE')
                self.finished = True
            elif s.mode != 'LAND' and self.can_send(2.0):
                print('[KOMUT] LAND moduna geçiliyor...')
                self.set_mode('LAND')
            elif (s.mode == 'LAND' and a is not None and a < GROUND_ALT
                  and time.time() - self.land_start > 10.0 and self.can_send(3.0)):
                # ArduPilot normalde yere değince kendisi disarm eder; olmazsa biz yollarız
                print('[KOMUT] Yerde ama hâlâ armed, disarm gönderiliyor...')
                req = CommandBool.Request()
                req.value = False
                self.send(self.arm_client, req, 'disarm')

    def emergency_land(self):
        """Ctrl+C durumunda en iyi çaba ile LAND moduna geç."""
        if self.state and self.state.armed:
            print('\n[ACİL] Ctrl+C: LAND moduna geçiliyor...')
            req = SetMode.Request()
            req.custom_mode = 'LAND'
            fut = self.mode_client.call_async(req)
            end = time.time() + 3.0
            while not fut.done() and time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.1)

    def print_status(self):
        if self.state is None:
            print('Bağlantı  : BEKLENİYOR...')
            return
        a, src, age = self.get_alt()
        print('═' * 40)
        print(f'Faz         : {self.phase}')
        print(f'Bağlantı    : {"BAĞLI" if self.state.connected else "KOPUK"}')
        print(f'Mod         : {self.state.mode}')
        print(f'Armed       : {self.state.armed}')
        if a is None:
            print('İrtifa      : VERİ YOK (hiçbir kaynaktan mesaj gelmiyor)')
        else:
            print(f'İrtifa      : {a:.2f} m  [{src}, {age:.1f} sn önce]')
        # Teşhis satırı: her kaynağın ham değeri
        raw = []
        for name in ('lidar',) + BARO_SOURCES:
            raw.append(f'{name}={self.src[name][0]:.2f}' if self.is_fresh(name)
                       else f'{name}=-')
        print(f'Ham veriler : {"  ".join(raw)}')
        print('═' * 40)


def main():
    start_mavros()
    rclpy.init()
    node = TakeoffTest()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.emergency_land()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
