#!/usr/bin/env python3
"""
MAVROS (ArduPilot / SITL) kalkış - navigasyon - iniş testi (v3).

Akış (durum makinesi):
  WAIT_CONNECT -> STREAMS -> SYNC -> GUIDED -> ARM -> TAKEOFF -> CLIMB
  -> NAVIGATE -> LAND -> DONE

Önemli noktalar:
  * STREAMS: ArduPilot seri/UDP bağlantıda veriyi kendiliğinden yollamaz,
    /mavros/set_stream_rate ile akış istenir.
  * SYNC: MAVROS bağlanınca parametre/mission/geofence indirir. Bu sırada hat
    meşgul olur, veri ve komutlar saniyelerce gecikebilir. ARM'dan önce hattın
    sakinleşmesi (veri kesintisiz akması) beklenir.
  * Havadayken veri kesintisi kademeli ele alınır (uyar/bekle -> LAND).
  * İrtifa, ARM anındaki z değerine göre (göreceli) hesaplanır.

Koordinat sistemi (MAVROS ENU - East North Up):
  x = Doğu (+ doğu, - batı) | y = Kuzey (+ kuzey, - güney) | z = Yukarı
Yaw (ENU): 0° = DOĞU, 90° = KUZEY, 180° = BATI, -90° = GÜNEY

bu kod gerçek hayatta denendi ve irtifa verisi geldi.
"""
import subprocess
import time
import math
import os
import signal
import atexit

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy, qos_profile_sensor_data)
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from geometry_msgs.msg import PoseStamped

# ========================= AYARLAR =========================
FCU_URL = 'udp://127.0.0.1:14550@14555'   # SITL
# FCU_URL = '/dev/ttyS0:115200'            # Gerçek drone (Pi <-> Pixhawk seri hattı)

# -- Kalkış --
TARGET_ALT = 3.0        # metre (kalkış yüksekliği, ARM noktasına göre)
ALT_TOLERANCE = 0.3     # hedef irtifaya bu kadar yaklaşınca "ulaştı" say
CLIMB_TIMEOUT = 30.0    # bu sürede irtifaya çıkamazsa inişe geç

# -- Navigasyon (kalkış noktasına GÖRE, metre) --
HEDEF_X = 0.0           # doğu-batı   (+ doğu)
HEDEF_Y = 5.0          # kuzey-güney (+ kuzey)
HEDEF_YAW = 90.0        # ENU: 0° Doğu, 90° Kuzey, 180° Batı, -90° Güney

WAYPOINT_TOLERANCE = 0.5  # hedefe bu kadar yaklaşınca "ulaştı" say (m)
NAVIGATE_TIMEOUT = 60.0   # navigasyon zaman aşımı (sn)

# -- Veri akışı --
STREAM_RATE_HZ = 10       # Pixhawk'tan istenen veri akış hızı (Hz)
ALT_DATA_TIMEOUT = 15.0   # ilk irtifa verisi için max bekleme (sn)

# -- SYNC fazı (ARM'dan önce hat sakinleşsin) --
SYNC_MIN_SECONDS = 15.0     # en az bu kadar bekle
SYNC_STABLE_SECONDS = 8.0   # bu süre boyunca veri kesintisiz gelmeli
STABLE_GAP = 1.0            # iki veri arası bu süreyi geçerse "kesinti" say
SYNC_MAX_SECONDS = 90.0     # kararlı hale gelmezse ARM etmeden çık
# Gerçek drone seri hattında (115200) parametre indirme çok daha uzun sürer.
# Orada SYNC_MAX_SECONDS'ı 180-240 yapmanız gerekebilir.

# -- Havadayken veri kesintisi --
STALE_WARN_SECONDS = 2.0    # bu kadar veri gelmezse uyar ve bekle
STALE_LAND_SECONDS = 10.0   # bu kadar gelmezse LAND dene
# NOT: Hat tamamen koparsa LAND komutu da gidemez (aynı hat). Asıl güvenlik
#      için FC'de failsafe parametrelerini (FS_GCS_ENABLE vb.) ayarlayın.

# -- İniş --
GROUND_ALT = 0.3            # bu irtifanın altı "yerde" sayılır (m)
GROUND_WAIT_SECONDS = 5.0   # yerde bu kadar durup hâlâ armed ise disarm gönder
# ===========================================================

mavros_proc = None


def yaw_derece_to_quaternion(yaw_derece):
    """ENU yaw açısını (derece) quaternion'a çevir (sadece z ekseni dönüşü)."""
    yaw_rad = math.radians(yaw_derece)
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def quaternion_to_yaw_derece(x, y, z, w):
    """Quaternion'dan ENU yaw açısını (derece) hesapla."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


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


class NavigasyonTest(Node):
    def __init__(self):
        super().__init__('mavros_nav_test')

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # ---- Drone durum verileri ----
        self.state = None
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.alt = 0.0            # EKF z (m), mutlak
        self.yaw = 0.0            # derece (ENU)
        self.alt_time = 0.0       # son pozisyon verisinin zamanı
        self.alt_ref = 0.0        # ARM anındaki z (göreceli ölçüm için)
        self.last_gap_time = 0.0  # son veri kesintisinin bittiği zaman

        # ---- Navigasyon hedefi ----
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = HEDEF_YAW
        self.navigating = False

        # ---- Faz yönetimi ----
        self.phase = 'WAIT_CONNECT'
        self.finished = False
        self.streams_start = 0.0
        self.sync_start = 0.0
        self.pending = None
        self.pending_tag = None
        self.last_req = 0.0
        self.last_warn = 0.0
        self.climb_start = 0.0
        self.navigate_start = 0.0
        self.ground_since = 0.0

        # ---- Abonelikler ----
        self.create_subscription(State, '/mavros/state', self.state_cb, state_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.pose_cb, qos_profile_sensor_data)

        # ---- Yayıncılar ----
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        # ---- Servis istemcileri ----
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.stream_client = self.create_client(StreamRate, '/mavros/set_stream_rate')

        for name, cli in (('arming', self.arm_client),
                          ('takeoff', self.takeoff_client),
                          ('set_mode', self.mode_client)):
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{name} servisi bekleniyor...')

        # ---- Zamanlayıcılar ----
        self.create_timer(0.2, self.tick)              # durum makinesi (5 Hz)
        self.create_timer(1.0, self.print_status)      # durum yazdırma (1 Hz)
        self.create_timer(0.1, self.publish_setpoint)  # setpoint (10 Hz)

    # ==================== CALLBACK'LER ====================

    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        now = time.time()
        # İki mesaj arasında büyük boşluk olduysa kesinti olarak işaretle
        if self.alt_time > 0.0 and (now - self.alt_time) > STABLE_GAP:
            self.last_gap_time = now
        self.alt_time = now

        self.pos_x = msg.pose.position.x
        self.pos_y = msg.pose.position.y
        self.alt = msg.pose.position.z

        o = msg.pose.orientation
        self.yaw = quaternion_to_yaw_derece(o.x, o.y, o.z, o.w)

    # ==================== SETPOINT YAYINLAMA ====================

    def publish_setpoint(self):
        """navigating True iken hedef pozisyon + yaw'ı 10 Hz'de Pixhawk'a gönderir."""
        if not self.navigating:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z

        qx, qy, qz, qw = yaw_derece_to_quaternion(self.target_yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.setpoint_pub.publish(msg)

    # ==================== YARDIMCILAR ====================

    def alt_age(self):
        """Son pozisyon verisinin yaşı (sn). Hiç gelmediyse sonsuz."""
        if self.alt_time <= 0.0:
            return float('inf')
        return time.time() - self.alt_time

    def rel_alt(self):
        """ARM noktasına göre irtifa (m)."""
        return self.alt - self.alt_ref

    def can_send(self, interval):
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

    def start_land(self):
        self.navigating = False
        self.ground_since = 0.0
        self.set_phase('LAND')

    def distance_to_target(self):
        dx = self.target_x - self.pos_x
        dy = self.target_y - self.pos_y
        dz = self.target_z - self.alt
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def handle_result(self, tag, res):
        if res is None:
            print(f'[HATA] {tag}: cevap alınamadı')
            return
        if tag == 'takeoff':
            if res.success:
                print(f'[SONUÇ] Kalkış komutu kabul edildi ({TARGET_ALT} m)')
                self.climb_start = time.time()
                self.set_phase('CLIMB')
            else:
                print('[SONUÇ] Kalkış komutu reddedildi, tekrar denenecek')
        elif tag == 'stream':
            print("[SONUÇ] Veri akışı isteği Pixhawk'a iletildi")
        elif tag == 'arm':
            print('[SONUÇ] Arm komutu kabul edildi' if res.success
                  else '[SONUÇ] Arm reddedildi (pre-arm: GPS/EKF hazır mı?), tekrar denenecek')
        elif tag == 'disarm':
            print('[SONUÇ] Disarm kabul edildi' if res.success
                  else '[SONUÇ] Disarm reddedildi')

    # ==================== DURUM MAKİNESİ ====================
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
        age = self.alt_age()

        # ---- Havadayken güvenlik kontrolleri ----
        if self.phase in ('CLIMB', 'NAVIGATE'):
            # Pilot modu değiştirdiyse kontrolü bırak
            if s.mode != 'GUIDED':
                print(f'[UYARI] Mod {s.mode} oldu (GUIDED değil). '
                      'Kontrol pilota bırakıldı, komut gönderilmiyor.')
                self.navigating = False
                self.set_phase('DONE')
                self.finished = True
                return
            # Uzun veri kesintisi: LAND
            if age > STALE_LAND_SECONDS:
                print(f'[UYARI] İrtifa verisi {age:.0f} sn kesildi! LAND moduna geçiliyor')
                self.start_land()
                return
            # Kısa veri kesintisi: uyar, karar verme, bekle
            if age > STALE_WARN_SECONDS:
                if time.time() - self.last_warn > 2.0:
                    print(f'[UYARI] Veri {age:.1f} sn gelmedi, bekleniyor '
                          f'({STALE_LAND_SECONDS:.0f} sn aşılırsa LAND)')
                    self.last_warn = time.time()
                return

        # -------- WAIT_CONNECT --------
        if self.phase == 'WAIT_CONNECT':
            if s.connected:
                self.streams_start = time.time()
                self.set_phase('STREAMS')

        # -------- STREAMS: veri akışını iste, ilk veri gelsin --------
        elif self.phase == 'STREAMS':
            if age < STALE_WARN_SECONDS:
                print(f'[BİLGİ] İrtifa verisi geliyor ({self.alt:.2f} m)')
                self.sync_start = time.time()
                self.last_gap_time = time.time()
                self.set_phase('SYNC')
            elif time.time() - self.streams_start > ALT_DATA_TIMEOUT:
                print('[HATA] İrtifa verisi gelmiyor! ARM EDİLMEDİ, çıkılıyor.')
                print('       Kontrol: ros2 topic hz /mavros/local_position/pose')
                print('       Pixhawk: SRx_POSITION / SRx_EXTRA1 / SRx_EXT_STAT parametreleri')
                self.finished = True
            elif self.can_send(3.0):
                if self.stream_client.service_is_ready():
                    req = StreamRate.Request()
                    req.stream_id = 0           # 0 = tüm akışlar
                    req.message_rate = STREAM_RATE_HZ
                    req.on_off = True
                    print(f'[KOMUT] Veri akışı isteniyor ({STREAM_RATE_HZ} Hz)...')
                    self.send(self.stream_client, req, 'stream')
                else:
                    print('[UYARI] /mavros/set_stream_rate servisi yok, veri kendiliğinden bekleniyor')
                    self.last_req = time.time()

        # -------- SYNC: parametre/mission indirmesi bitsin, hat sakinleşsin --------
        elif self.phase == 'SYNC':
            now = time.time()
            elapsed = now - self.sync_start
            stable = (age < STABLE_GAP and
                      (now - self.last_gap_time) >= SYNC_STABLE_SECONDS)
            if elapsed >= SYNC_MIN_SECONDS and stable:
                print(f'[BİLGİ] Veri akışı kararlı ({elapsed:.0f} sn), ARM aşamasına geçiliyor')
                self.set_phase('GUIDED')
            elif elapsed > SYNC_MAX_SECONDS:
                print('[HATA] Veri akışı kararlı hale gelmedi! ARM EDİLMEDİ, çıkılıyor.')
                self.finished = True

        # -------- GUIDED --------
        elif self.phase == 'GUIDED':
            if s.mode == 'GUIDED':
                self.set_phase('ARM')
            elif self.can_send(2.0):
                print('[KOMUT] GUIDED moduna geçiliyor...')
                self.set_mode('GUIDED')

        # -------- ARM --------
        elif self.phase == 'ARM':
            if s.armed:
                self.alt_ref = self.alt     # yerdeki irtifayı sıfır noktası yap
                print(f'[BİLGİ] İrtifa sıfır noktası: z_ref={self.alt_ref:.2f}')
                self.set_phase('TAKEOFF')
            elif self.can_send(3.0):
                print('[KOMUT] Arm isteği gönderiliyor...')
                req = CommandBool.Request()
                req.value = True
                self.send(self.arm_client, req, 'arm')

        # -------- TAKEOFF --------
        elif self.phase == 'TAKEOFF':
            if not s.armed:
                print('[UYARI] Dron disarm oldu, arm aşamasına dönülüyor')
                self.set_phase('ARM')
            elif self.can_send(1.0):
                print(f'[KOMUT] Kalkış: {TARGET_ALT} m')
                req = CommandTOL.Request()
                req.altitude = float(TARGET_ALT)
                self.send(self.takeoff_client, req, 'takeoff')

        # -------- CLIMB --------
        elif self.phase == 'CLIMB':
            if self.rel_alt() >= TARGET_ALT - ALT_TOLERANCE:
                self.target_x = self.pos_x + HEDEF_X
                self.target_y = self.pos_y + HEDEF_Y
                # Tam hedef irtifa: ARM noktası + TARGET_ALT (anlık değere eşitleme yok)
                self.target_z = self.alt_ref + TARGET_ALT

                self.navigate_start = time.time()
                self.navigating = True

                toplam_mesafe = math.sqrt(HEDEF_X ** 2 + HEDEF_Y ** 2)
                print(f'[BİLGİ] Hedef irtifaya ulaşıldı ({self.rel_alt():.2f} m)')
                print(f'[NAV]   Şu anki konum : x={self.pos_x:.2f}  y={self.pos_y:.2f}  z={self.rel_alt():.2f}')
                print(f'[NAV]   Hedef konum   : x={self.target_x:.2f}  y={self.target_y:.2f}  z={TARGET_ALT:.2f}')
                print(f'[NAV]   Toplam mesafe : {toplam_mesafe:.1f} m')
                self.set_phase('NAVIGATE')

            elif time.time() - self.climb_start > CLIMB_TIMEOUT:
                print('[UYARI] İrtifaya çıkma zaman aşımı, iniş başlatılıyor')
                self.start_land()

        # -------- NAVIGATE --------
        elif self.phase == 'NAVIGATE':
            dist = self.distance_to_target()

            if dist < WAYPOINT_TOLERANCE:
                print(f'[NAV] ✓ Hedefe ulaşıldı! (kalan mesafe: {dist:.2f} m)')
                print(f'[NAV]   Son konum: x={self.pos_x:.2f}  y={self.pos_y:.2f}  z={self.rel_alt():.2f}')
                self.start_land()

            elif time.time() - self.navigate_start > NAVIGATE_TIMEOUT:
                print(f'[UYARI] Navigasyon zaman aşımı! (kalan mesafe: {dist:.2f} m)')
                self.start_land()

        # -------- LAND --------
        elif self.phase == 'LAND':
            if not s.armed:
                print('[SONUÇ] DRON İNDİ VE DISARM OLDU!')
                self.set_phase('DONE')
                self.finished = True
            elif s.mode != 'LAND':
                if self.can_send(2.0):
                    print('[KOMUT] LAND moduna geçiliyor...')
                    self.set_mode('LAND')
            else:
                # LAND modunda: ArduPilot yere değince kendisi disarm eder.
                # Yedek olarak: yerde GROUND_WAIT_SECONDS durup hâlâ armed ise disarm yolla.
                on_ground = age < STALE_WARN_SECONDS and self.rel_alt() < GROUND_ALT
                if not on_ground:
                    self.ground_since = 0.0
                elif self.ground_since == 0.0:
                    self.ground_since = time.time()
                elif (time.time() - self.ground_since > GROUND_WAIT_SECONDS
                      and self.can_send(3.0)):
                    print('[KOMUT] Yerde ama hâlâ armed, disarm gönderiliyor...')
                    req = CommandBool.Request()
                    req.value = False
                    self.send(self.arm_client, req, 'disarm')

    def emergency_land(self):
        """Ctrl+C durumunda en iyi çaba ile LAND moduna geç."""
        self.navigating = False
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
        age = self.alt_age()
        veri = 'güncel' if age < STALE_WARN_SECONDS else f'VERİ YOK ({age:.1f} sn)'
        print('═' * 60)
        print(f'Faz         : {self.phase}')
        print(f'Bağlantı    : {"BAĞLI" if self.state.connected else "KOPUK"}')
        print(f'Mod         : {self.state.mode}')
        print(f'Armed       : {self.state.armed}')
        print(f'Pozisyon    : x={self.pos_x:+.2f}  y={self.pos_y:+.2f}  z={self.rel_alt():.2f} m ({veri})')
        print(f'Yaw (ENU)   : {self.yaw:+.1f}° (hedef: {self.target_yaw:.1f}°)')
        if self.phase == 'SYNC':
            now = time.time()
            print(f'Senkron     : {now - self.sync_start:.0f} sn, '
                  f'kesintisiz süre: {now - self.last_gap_time:.0f}/{SYNC_STABLE_SECONDS:.0f} sn')
        if self.navigating:
            print(f'Hedef       : x={self.target_x:+.2f}  y={self.target_y:+.2f}  '
                  f'z={self.target_z - self.alt_ref:.2f} m (kalkışa göre)')
            print(f'Kalan mesafe: {self.distance_to_target():.2f} m')
        print('═' * 60)


def main():
    start_mavros()
    rclpy.init()
    node = NavigasyonTest()
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
