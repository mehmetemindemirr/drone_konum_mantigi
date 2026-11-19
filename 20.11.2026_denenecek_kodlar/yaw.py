#!/usr/bin/env python3
"""
MAVROS (ArduPilot / SITL) kalkış - navigasyon - iniş testi.

deneme.py'nin üzerine x, y, z pozisyon kontrolü eklenmiş hali.
Drone kalkış yapar, belirlenen x/y/z kadar ilerler, sonra iner.

Akış (durum makinesi):
  WAIT_CONNECT -> STREAMS -> GUIDED -> ARM -> TAKEOFF -> CLIMB -> NAVIGATE -> LAND -> DONE

Koordinat Sistemi (MAVROS ENU - East North Up):
  ┌─────────────────────────────────────────────────┐
  │                                                 │
  │        y (Kuzey)                                │
  │        ▲                                        │
  │        │                                        │
  │        │    Drone                               │
  │        │   ◈ ─ ─ ─ → x (Doğu)                  │
  │        │                                        │
  │        └──────────────►                         │
  │       ╱                                         │
  │      ╱                                          │
  │     z (Yukarı)                                  │
  │                                                 │
  │  x = Doğu yönü   (+ doğu,  - batı)             │
  │  y = Kuzey yönü   (+ kuzey, - güney)            │
  │  z = Yükseklik    (+ yukarı, - aşağı)           │
  │                                                 │
  │  Orijin (0,0,0) = EKF başlangıç noktası         │
  │                    (genelde arm edildiği yer)     │
  └─────────────────────────────────────────────────┘

Örnek: HEDEF_X=5, HEDEF_Y=0 → drone doğuya 5 metre gider
        HEDEF_X=0, HEDEF_Y=10 → drone kuzeye 10 metre gider
        HEDEF_X=5, HEDEF_Y=5  → drone çapraz (kuzeydoğu) gider
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
#FCU_URL = '/dev/ttyS0:115200'              # Gerçek drone (Pi <-> Pixhawk seri hattı)

# -- Kalkış --
TARGET_ALT = 15.0        # metre (kalkış yüksekliği)
ALT_TOLERANCE = 0.3     # hedef irtifaya bu kadar yaklaşınca "ulaştı" say
CLIMB_TIMEOUT = 40.0    # bu sürede irtifaya çıkamazsa inişe geç

# -- Navigasyon (X, Y, Z hedefleri) --
# Kalkış noktasına GÖRE ne kadar ilerleyeceği (metre)
# Örn: HEDEF_X=5  → doğuya 5m,  HEDEF_Y=0 → kuzeye 0m
HEDEF_X = 0.0           # x ekseni: doğu-batı  (+ doğu)
HEDEF_Y = 10.0           # y ekseni: kuzey-güney (+ kuzey)
# z ekseni: kalkış yüksekliği TARGET_ALT ile belirleniyor, ayrıca ayarlamaya gerek yok

# -- Yaw (Drone'un baktığı yön) --
#   0° = Kuzey,  90° = Doğu,  180° = Güney,  -90° (veya 270°) = Batı
HEDEF_YAW = 180.0        # derece – hedef noktada drone hangi yöne baksın (90° = doğu)

WAYPOINT_TOLERANCE = 0.5  # hedef noktaya bu kadar yaklaşınca "ulaştı" say (m)
NAVIGATE_TIMEOUT = 60.0   # navigasyon zaman aşımı (sn)

# -- Veri Akışı --
STREAM_RATE_HZ = 10     # Pixhawk'tan istenen veri akış hızı (Hz)
ALT_STALE_SECONDS = 10.0 # irtifa verisi bu süreden eski ise "veri yok" say
ALT_DATA_TIMEOUT = 15.0 # kalkıştan önce irtifa verisi için max bekleme
# ===========================================================

mavros_proc = None


def yaw_derece_to_quaternion(yaw_derece):
    """
    Yaw açısını (derece) quaternion formatına çevir.

    Pixhawk oryantasyonu quaternion olarak bekler.
    Quaternion = (x, y, z, w) → 4 sayıyla 3D dönüşü ifade eder.
    Biz sadece yaw (sağa-sola dönüş) kullandığımız için
    x ve y her zaman 0, sadece z ve w değişir.

    Derece → Quaternion örneği:
      0°   → (0, 0,  0.00,  1.00)  = Kuzey
      90°  → (0, 0,  0.71,  0.71)  = Doğu
      180° → (0, 0,  1.00,  0.00)  = Güney
      -90° → (0, 0, -0.71,  0.71)  = Batı
    """
    yaw_rad = math.radians(yaw_derece)
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return qx, qy, qz, qw


def quaternion_to_yaw_derece(x, y, z, w):
    """
    Quaternion'dan yaw açısını (derece) geri hesapla.
    Pixhawk'tan gelen oryantasyonu okunabilir açıya çevirmek için kullanılır.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_rad)


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

        # ---- Drone durum verileri (Pixhawk'tan sürekli güncellenir) ----
        self.state = None
        self.pos_x = 0.0         # yerel x konumu (doğu, metre)
        self.pos_y = 0.0         # yerel y konumu (kuzey, metre)
        self.alt = 0.0           # yerel z konumu (yukarı, metre) = irtifa
        self.yaw = 0.0           # drone'un baktığı yön (derece, 0°=kuzey)
        self.alt_time = 0.0      # son pozisyon verisinin zamanı
        self.alt_ref = 0.0       # kalkış anındaki z (göreceli ölçüm için)

        # ---- Navigasyon hedefi ----
        # Bu değerler CLIMB fazı bitince hesaplanır:
        #   target_x = şu_anki_x + HEDEF_X
        #   target_y = şu_anki_y + HEDEF_Y
        # Yani "kalkış noktasından 5m doğuya" gibi.
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = HEDEF_YAW  # hedef yaw açısı (derece)

        # navigating = True iken publish_setpoint() çalışır
        # ve Pixhawk'a sürekli "bu noktaya git" komutu gönderir.
        # False iken hiçbir komut gönderilmez.
        self.navigating = False

        # ---- Faz yönetimi ----
        self.phase = 'WAIT_CONNECT'
        self.finished = False
        self.streams_start = 0.0
        self.pending = None
        self.pending_tag = None
        self.last_req = 0.0
        self.climb_start = 0.0
        self.navigate_start = 0.0
        self.land_start = 0.0

        # ---- Abonelikler (subscribers) ----
        self.create_subscription(State, '/mavros/state', self.state_cb, state_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.pose_cb, qos_profile_sensor_data)

        # ---- Yayıncılar (publishers) ----
        # Drone'a "buraya git" demek için bu topic'e PoseStamped yayınlıyoruz.
        # MAVROS bunu Pixhawk'a MAVLink SET_POSITION_TARGET olarak iletir.
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
        self.create_timer(0.2, self.tick)            # durum makinesi (5 Hz)
        self.create_timer(1.0, self.print_status)    # durum yazdırma (1 Hz)
        self.create_timer(0.1, self.publish_setpoint) # pozisyon yayınlama (10 Hz)

    # ==================== CALLBACK'LER ====================

    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        """
        /mavros/local_position/pose callback'i.

        Pixhawk'ın EKF'inden gelen yerel pozisyon:
          msg.pose.position.x    → Doğu konumu  (metre)
          msg.pose.position.y    → Kuzey konumu  (metre)
          msg.pose.position.z    → Yükseklik     (metre)
          msg.pose.orientation.* → Drone'un yönü (quaternion)
        """
        self.pos_x = msg.pose.position.x
        self.pos_y = msg.pose.position.y
        self.alt = msg.pose.position.z
        self.alt_time = time.time()

        # Quaternion → derece çevir (drone şu an hangi yöne bakıyor?)
        o = msg.pose.orientation
        self.yaw = quaternion_to_yaw_derece(o.x, o.y, o.z, o.w)

    # ==================== SETPOINT YAYINLAMA ====================
    #
    # SETPOINT NEDİR?
    # ───────────────
    # HEDEF_X = 5 yazmak sadece bir AYAR. Drone bunu bilmez!
    # Drone'un hareket etmesi için bu ayarı Pixhawk'a İLETMEMİZ lazım.
    #
    # İşte publish_setpoint() tam bunu yapıyor:
    #   Sen: HEDEF_X = 5 ("5 metre doğuya git istiyorum")
    #   Kod: target_x = pos_x + 5 hesaplar
    #   publish_setpoint(): Bu target_x'i Pixhawk'a gönderir (saniyede 10 kez)
    #   Pixhawk: Motorları çalıştırıp drone'u oraya götürür
    #
    # Kısaca: HEDEF_X = nereye gitmek istediğin
    #         publish_setpoint = Pixhawk'a "git şuraya!" demek
    #

    def publish_setpoint(self):
        """
        Hedef pozisyonu + yaw açısını Pixhawk'a gönder.

        Bu fonksiyon 10 Hz'de (saniyede 10 kez) çalışır.
        self.navigating True iken hedef koordinatları Pixhawk'a yayınlar.

        NEDEN SÜREKLİ YAYINLIYORUZ?
        1. Pixhawk setpoint kesilirse güvenlik moduna geçebilir
        2. Rüzgâr düzeltmeleri anlık uygulanır
        3. Sürüde: her drone kendi hedefini sürekli günceller
        """
        if not self.navigating:
            return

        # --- Pozisyon ---
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.target_x    # hedef x (doğu)
        msg.pose.position.y = self.target_y    # hedef y (kuzey)
        msg.pose.position.z = self.target_z    # hedef z (yükseklik)

        # --- Yaw (drone'un bakacağı yön) ---
        qx, qy, qz, qw = yaw_derece_to_quaternion(self.target_yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.setpoint_pub.publish(msg)

    # ==================== YARDIMCILAR ====================

    def alt_fresh(self):
        """İrtifa verisi yakın zamanda geldi mi? (Hiç gelmediyse False)"""
        return self.alt_time > 0.0 and (time.time() - self.alt_time) < ALT_STALE_SECONDS

    def rel_alt(self):
        """Kalkış noktasına göre irtifa (m)."""
        return self.alt - self.alt_ref

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

    def distance_to_target(self):
        """Hedefe olan 3 boyutlu mesafe (metre)."""
        dx = self.target_x - self.pos_x
        dy = self.target_y - self.pos_y
        dz = self.target_z - self.alt
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def handle_result(self, tag, res):
        if res is None:
            print(f'[HATA] {tag}: cevap alınamadı')
            return
        if tag == 'takeoff':
            if res.success:
                print(f'[SONUÇ] Kalkış komutu kabul edildi ({TARGET_ALT} m)')
                self.alt_ref = self.alt
                self.climb_start = time.time()
                self.set_phase('CLIMB')
            else:
                print('[SONUÇ] Kalkış komutu reddedildi, tekrar denenecek')
        elif tag == 'stream':
            print('[SONUÇ] Veri akışı isteği Pixhawk\'a iletildi')
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

        # Havadayken veri kesilirse körü körüne devam etme, hemen in
        if self.phase in ('CLIMB', 'NAVIGATE') and not self.alt_fresh():
            print('[UYARI] İrtifa verisi kesildi! LAND moduna geçiliyor')
            self.navigating = False
            self.land_start = time.time()
            self.set_phase('LAND')
            return

        # -------- WAIT_CONNECT --------
        if self.phase == 'WAIT_CONNECT':
            if s.connected:
                self.streams_start = time.time()
                self.set_phase('STREAMS')

        # -------- STREAMS --------
        elif self.phase == 'STREAMS':
            if self.alt_fresh():
                print(f'[BİLGİ] İrtifa verisi geliyor ({self.alt:.2f} m)')
                self.set_phase('GUIDED')
            elif time.time() - self.streams_start > ALT_DATA_TIMEOUT:
                print('[HATA] İrtifa verisi gelmiyor! ARM EDİLMEDİ, çıkılıyor.')
                print('       Kontrol: ros2 topic hz /mavros/local_position/pose')
                print('       Pixhawk: SRx_POSITION / SRx_EXTRA1 / SRx_EXT_STAT parametreleri')
                self.finished = True
            elif self.can_send(3.0):
                if self.stream_client.service_is_ready():
                    req = StreamRate.Request()
                    req.stream_id = 0
                    req.message_rate = STREAM_RATE_HZ
                    req.on_off = True
                    print(f'[KOMUT] Veri akışı isteniyor ({STREAM_RATE_HZ} Hz)...')
                    self.send(self.stream_client, req, 'stream')
                else:
                    print('[UYARI] /mavros/set_stream_rate servisi yok, veri kendiliğinden bekleniyor')
                    self.last_req = time.time()

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
                # İrtifaya ulaştık! Şimdi navigasyon hedefini hesapla.
                #
                # Kalkış noktasındaki (x, y) konumuna HEDEF_X ve HEDEF_Y ekle.
                # Böylece drone kalkış noktasından belirtilen kadar ilerler.
                #
                # Örnek: Şu an x=2, y=1 ve HEDEF_X=5, HEDEF_Y=0 ise
                #        hedef = (2+5, 1+0, mevcut_irtifa) = (7, 1, 5)
                #        yani doğuya 5 metre daha gider
                #
                self.target_x = self.pos_x + HEDEF_X
                self.target_y = self.pos_y + HEDEF_Y
                self.target_z = self.alt   # aynı yükseklikte kal

                self.navigate_start = time.time()
                self.navigating = True     # setpoint yayınlamayı başlat!

                toplam_mesafe = math.sqrt(HEDEF_X**2 + HEDEF_Y**2)

                print(f'[BİLGİ] Hedef irtifaya ulaşıldı ({self.rel_alt():.2f} m)')
                print(f'[NAV]   Şu anki konum : x={self.pos_x:.2f}  y={self.pos_y:.2f}  z={self.alt:.2f}')
                print(f'[NAV]   Hedef konum   : x={self.target_x:.2f}  y={self.target_y:.2f}  z={self.target_z:.2f}')
                print(f'[NAV]   Toplam mesafe  : {toplam_mesafe:.1f} m')
                self.set_phase('NAVIGATE')

            elif time.time() - self.climb_start > CLIMB_TIMEOUT:
                print('[UYARI] İrtifaya çıkma zaman aşımı, iniş başlatılıyor')
                self.set_phase('LAND')

        # -------- NAVIGATE --------
        elif self.phase == 'NAVIGATE':
            # Hedefe olan mesafeyi kontrol et
            dist = self.distance_to_target()

            if dist < WAYPOINT_TOLERANCE:
                # Hedefe ulaştık!
                print(f'[NAV] ✓ Hedefe ulaşıldı! (kalan mesafe: {dist:.2f} m)')
                print(f'[NAV]   Son konum: x={self.pos_x:.2f}  y={self.pos_y:.2f}  z={self.alt:.2f}')
                self.navigating = False    # setpoint yayınlamayı durdur
                self.land_start = time.time()
                self.set_phase('LAND')

            elif time.time() - self.navigate_start > NAVIGATE_TIMEOUT:
                print(f'[UYARI] Navigasyon zaman aşımı! (kalan mesafe: {dist:.2f} m)')
                self.navigating = False
                self.land_start = time.time()
                self.set_phase('LAND')

        # -------- LAND --------
        elif self.phase == 'LAND':
            if not self.land_start:
                self.land_start = time.time()
            if not s.armed:
                print('[SONUÇ] DRON İNDİ VE DISARM OLDU!')
                self.set_phase('DONE')
                self.finished = True
            elif s.mode != 'LAND' and self.can_send(2.0):
                print('[KOMUT] LAND moduna geçiliyor...')
                self.set_mode('LAND')
            elif (s.mode == 'LAND' and self.alt_fresh() and self.rel_alt() < 0.3
                  and time.time() - self.land_start > 10.0 and self.can_send(3.0)):
                print('[KOMUT] Yerde ama hâlâ armed, disarm gönderiliyor...')
                req = CommandBool.Request()
                req.value = False
                self.send(self.arm_client, req, 'disarm')

    def emergency_land(self):
        """Ctrl+C durumunda en iyi çaba ile LAND moduna geç."""
        self.navigating = False  # setpoint yayınlamayı durdur
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
        veri = 'güncel' if self.alt_fresh() else 'VERİ YOK'
        print('═' * 60)
        print(f'Faz         : {self.phase}')
        print(f'Bağlantı    : {"BAĞLI" if self.state.connected else "KOPUK"}')
        print(f'Mod         : {self.state.mode}')
        print(f'Armed       : {self.state.armed}')
        print(f'Pozisyon    : x={self.pos_x:+.2f}  y={self.pos_y:+.2f}  z={self.rel_alt():.2f} m ({veri})')
        print(f'Yaw         : {self.yaw:+.1f}° (hedef: {self.target_yaw:.1f}°)')
        if self.navigating:
            dist = self.distance_to_target()
            print(f'Hedef       : x={self.target_x:+.2f}  y={self.target_y:+.2f}  z={self.target_z:.2f} m')
            print(f'Kalan mesafe: {dist:.2f} m')
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
