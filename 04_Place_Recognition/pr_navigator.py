"""
PR-Navigator: Standalone-Node fuer Place-Recognition-Startmodus.

Erkennt die Startposition (P2 / P4 / P6 / P9) per Place Recognition,
berechnet das aktuelle Heading via Cross-Korrelation und faehrt die
jeweils naechsten ZWEI Wegpunkte ab:

  P2 → WP3-Sued(0.40m) → MCL-Westdrehung + Wandkorrektur →
       WP3(0.60m) → P4(0.25m)   [MCL-Logik mit _front_wall_angle + Wandkorrektur]
  P4 → WP5-Nord(0.60m) → +90° West → WP6/P6  (1.59m)
  P6 → WP7-Sued(0.32m) → -90° West → WP8     (0.35m)
  P9 → WP10-Nord(0.68m) → +90° West → WP11   (0.215m)

Kein MCL, kein Partikelfilter – nur Lidar (Frontstopp) + IMU-korrigierter
Erstturn + Teensy-Kommandos.

Starten:
    ros2 launch mcl_nav pr_nav.launch.py
oder:
    ros2 run mcl_nav pr_navigator
(RPLidar und BNO055 muessen bereits laufen)
"""

import math
import os
import sys
import time
import warnings
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32, String

# ── Place-Recognition-Hilfsfunktionen ────────────────────────────────────────
_PR_DEV = '/home/pi/ros2_ws/src/mcl_nav/place_recognition_dev'
if _PR_DEV not in sys.path:
    sys.path.insert(0, _PR_DEV)
from place_signature import load_signature, estimate_rotation_shift, shift_to_degrees  # noqa: E402
from recognize_location import recognize, load_all_signatures              # noqa: E402

# ── Konfiguration ─────────────────────────────────────────────────────────────
SIGNATURE_DIR   = os.path.join(_PR_DEV, 'signatures')
PR_N_SCANS      = 10       # Scans mitteln (identisch zu learn_location.py)
PR_THRESHOLD    = 2253.0   # Dk-Schwellenwert (aus validate_offline.py)
PR_TOP_K        = 2        # Kandidaten fuer Feinvergleich

SEGMENT_LEN     = 0.0800   # m – Fahrsegmente (kleine Schitte fuer haeufige Frontstopp-Pruefung)
TEENSY_SCALE    = 1.03     # Teensy-Winkelskalierungsfaktor (empirisch)
TIMER_PERIOD    = 0.10     # s – Regelschritt

# Frontstopp-Abstaende 1. Wegpunkt je Startposition (aus map_config.yaml)
FRONT_STOP = {'P2': 0.40, 'P4': 0.60, 'P6': 0.32, 'P9': 0.68}

# 2. Wegpunkt: Drehung und Frontstopp nach Erreichen des 1. Wegpunkts
#   P2→WP3-Süd(0.40m) → -90° West → WP4/P4  (wp4_west_stop_dist)
#   P4→WP5-Nord(0.60m) → +90° West → WP6/P6  (front_stop_wp6)
#   P6→WP7-Süd(0.32m) → -90° West → WP8     (front_stop_wp8)
#   P9→WP10-Nord(0.68m) → +90° West → WP11   (front_stop_wp11)
SECOND_TURN_DEG = {'P4': +90.0, 'P6': -90.0, 'P9': +90.0}
SECOND_STOP     = {'P4':  1.590, 'P6':  0.35, 'P9':  0.215}

# Empirischer Offset P2-Suedturn (aus Hardware-Tests: +3° behebt Winkelunterschaetzung)
P2_SOUTH_TURN_OFFSET_DEG = 3.0

# ── P2-spezifisch: MCL-Logik WP3-West → P4 ──────────────────────────────────
# Werte identisch zu map_config.yaml (navigator.py)
WP3_WEST_STOP   = 0.60    # m  Frontstopp bei WP3 (trenner_unten sichtbar ~7.6° Sued v. West)
P4_WEST_STOP    = 0.25    # m  Frontstopp bei P4  (trenner_unten, wp4_west_stop_dist)
WP3_WALL_TARGET = 0.36    # m  Sollabstand untere_wand links (wp3_wall_target_dist)
WP3_WALL_GAIN   = 30.0    # °/m Wandkorrektur-Verstarkung (wp3_wall_angle_gain)


class State(Enum):
    INIT       = auto()   # Warte auf Sensoren
    PR_SCAN    = auto()   # Scans sammeln + PR durchfuehren
    PR_TURN    = auto()   # Warte auf T:-Feedback (Initialausrichtung 1. WP)
    PR_DRIVE   = auto()   # Fahrt in Segmenten → 1. Wegpunkt
    # P2-spezifisch: MCL-Logik WP3-West → P4
    WP3_TURN   = auto()   # P2: Warte auf T: nach front_wall_angle()-Westdrehung
    WP3_ALIGN  = auto()   # P2: Warte auf T: nach Wandkorrektur-Feinausrichtung
    WP3_WEST   = auto()   # P2: Westfahrt mit Wandkorrektur → WP3 (0.60m)
    P4_WEST    = auto()   # P2: Westfahrt mit Wandkorrektur → P4  (0.25m)
    # Generisch (P4/P6/P9)
    SEG2_TURN  = auto()   # Warte auf T:-Feedback (Drehung vor 2. WP)
    SEG2_DRIVE = auto()   # Fahrt in Segmenten → 2. Wegpunkt
    DONE       = auto()   # Alle Wegpunkte erreicht


class PRNavigator(Node):
    """
    Standalone-Node fuer PR-Startmodus.
    Erbt NICHTS vom MCL-Navigator – komplett unabhaengig.
    """

    def __init__(self):
        super().__init__('pr_navigator')

        # ── Subscriptions ────────────────────────────────────────────────
        self._sub_scan = self.create_subscription(
            LaserScan, '/scan', self._scan_cb,
            QoSPresetProfiles.SENSOR_DATA.value)
        self._sub_imu = self.create_subscription(
            Imu, '/imu/data', self._imu_cb, 10)
        self._sub_fb = self.create_subscription(
            String, 'odom_feedback', self._feedback_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────
        # Identische Topics wie navigator.py
        self._pub_cmd  = self.create_publisher(String, 'teensy_cmd',                  10)
        self._pub_stop = self.create_publisher(Int32,  'micro_ros_fabinal_subscriber', 10)

        # ── Sensorzustand ────────────────────────────────────────────────
        self._scan_ranges: np.ndarray | None = None
        self._scan_angles: np.ndarray | None = None
        self._imu_theta_raw: float | None = None
        self._imu_east_ref: float = 0.0          # BNO055-Wert bei Ost-Ausrichtung
        self._imu_initialized: bool = False

        # ── Feedback-Zustand ─────────────────────────────────────────────
        self._feedback_received: bool = False
        self._feedback_value: str | None = None

        # ── PR-Zustand ───────────────────────────────────────────────────
        self._pr_scan_buffer: list = []
        self._pr_stored: dict = {}
        self._pr_n_bins: int = 360

        # ── Fahrt-Parameter (nach PR-Erkennung befuellt) ─────────────────
        self._detected_loc: str | None = None       # z.B. 'P9'
        self._front_stop_dist: float | None = None

        # ── Zustandsmaschine ─────────────────────────────────────────────
        self.state = State.INIT

        # ── Signaturen laden ─────────────────────────────────────────────
        self._load_signatures()

        # ── Regelschritt-Timer ───────────────────────────────────────────
        self._timer = self.create_timer(TIMER_PERIOD, self._loop)
        self.get_logger().info('PR-Navigator gestartet. Warte auf Sensoren...')

    # ─────────────────────────────────────────────────────────────────────────
    # Sensor-Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        valid  = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges[~valid] = np.inf
        self._scan_ranges = ranges
        self._scan_angles = angles

    def _imu_cb(self, msg: Imu):
        q    = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw  = math.atan2(siny, cosy)
        self._imu_theta_raw = yaw
        if not self._imu_initialized:
            self._imu_east_ref   = yaw   # Startwert = unkalibriert, aber konsistent
            self._imu_initialized = True
            self.get_logger().info(
                f'IMU initialisiert: raw={math.degrees(yaw):.1f}°  '
                f'(imu_east_ref={math.degrees(self._imu_east_ref):.1f}°)')

    def _feedback_cb(self, msg: String):
        data = msg.data
        # R:ok / C:ok sind Bestaetigung des Empfangs, kein Fertig-Signal
        if data.startswith('R:') or data.startswith('C:'):
            return
        self._feedback_received = True
        self._feedback_value    = data
        self.get_logger().info(f'Feedback: {data}')

    # ─────────────────────────────────────────────────────────────────────────
    # Hilfsfunktionen
    # ─────────────────────────────────────────────────────────────────────────

    def _send_cmd(self, cmd: str):
        self.get_logger().info(f'CMD: {cmd}')
        msg      = String()
        msg.data = cmd
        self._pub_cmd.publish(msg)

    def _stop(self):
        """Notstopp: setzt Teensy auf neutrale Stellung (identisch zu navigator.py)."""
        msg      = Int32()
        msg.data = 500
        self._pub_stop.publish(msg)

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Kuerzester Winkelunterschied a − b in [-π, π]."""
        return (a - b + math.pi) % (2.0 * math.pi) - math.pi

    def _front_wall_dist(self, half_angle_deg: float = 15.0) -> float | None:
        """Minimaler Abstand zur Vorderwand (Winkel 0° ± half_angle_deg).
        Identische Implementierung wie navigator.py (Winkelfilter statt Bin-Index)."""
        if self._scan_ranges is None or self._scan_angles is None:
            return None
        angles = self._scan_angles
        ranges = self._scan_ranges
        mask   = (
            (angles > np.deg2rad(-half_angle_deg)) &
            (angles < np.deg2rad(half_angle_deg)) &
            np.isfinite(ranges) &
            (ranges < 3.0))
        if np.sum(mask) < 3:
            return None
        return float(np.min(ranges[mask]))

    def _front_wall_angle(self, half_angle_deg: float = 25.0) -> float:
        """Winkel des naechsten Punktes zur Vorderwand (identisch navigator.py).
        Positiv = Wand leicht rechts von 0°. Fallback 0° wenn zu wenig Punkte."""
        if self._scan_ranges is None or self._scan_angles is None:
            return 0.0
        angles = self._scan_angles
        ranges = self._scan_ranges
        mask = (
            (angles > np.deg2rad(-half_angle_deg)) &
            (angles < np.deg2rad(half_angle_deg)) &
            np.isfinite(ranges) &
            (ranges < 3.0))
        if np.sum(mask) < 3:
            self.get_logger().warn('_front_wall_angle: zu wenig Punkte, Fallback 0°')
            return 0.0
        masked_ranges = ranges.copy()
        masked_ranges[~mask] = np.inf
        idx   = int(np.argmin(masked_ranges))
        angle = float(np.degrees(angles[idx]))
        self.get_logger().info(
            f'_front_wall_angle: {angle:.1f}° (naechster Punkt {ranges[idx]:.3f}m)')
        return angle

    def _left_wall_min(self, half_angle_deg: float = 15.0):
        """Minimaler Abstand zur linken Wand (75°–105° bei half=15°).
        Bei Westfahrt = untere_wand (Suedwand). Identisch navigator.py."""
        if self._scan_ranges is None or self._scan_angles is None:
            return None, None
        angles = self._scan_angles
        ranges = self._scan_ranges
        mask = (
            (angles > np.deg2rad(90.0 - half_angle_deg)) &
            (angles < np.deg2rad(90.0 + half_angle_deg)) &
            np.isfinite(ranges) &
            (ranges < 3.0))
        if np.sum(mask) < 3:
            return None, None
        idx       = int(np.argmin(ranges[mask]))
        min_dist  = float(ranges[mask][idx])
        min_angle = float(np.degrees(angles[mask][idx]))
        return min_dist, min_angle

    def _wall_correction_wp3(self) -> bool:
        """Wandkorrektur fuer P2-Westfahrt: untere_wand links (75°–105°).
        Identisch _wall_angle_correction_wp3_west() in navigator.py.
        Gibt True zurueck wenn T: gesendet wurde."""
        min_dist, _ = self._left_wall_min(half_angle_deg=15)
        if min_dist is None:
            return False
        dist_err = min_dist - WP3_WALL_TARGET
        turn_deg = dist_err * WP3_WALL_GAIN
        turn_deg = max(-15.0, min(15.0, turn_deg))
        self.get_logger().info(
            f'WP3-Wandkorr: dist={min_dist:.3f}m  err={dist_err*100:.1f}cm  T={turn_deg:.1f}°')
        if abs(turn_deg) > 0.5:
            self._send_cmd(f'T:{turn_deg:.1f}')
            return True
        return False

    def _scan_to_angle_signature(self) -> np.ndarray | None:
        """Aktuellen Scan in n_bins-Winkel-Signatur umwandeln (identisch zu learn_location.py)."""
        if self._scan_ranges is None or self._scan_angles is None:
            return None
        n_bins     = self._pr_n_bins
        binned     = np.full(n_bins, np.inf)
        angles_deg = np.degrees(self._scan_angles) % 360.0
        for i in range(len(self._scan_ranges)):
            r = float(self._scan_ranges[i])
            if np.isfinite(r) and 0.1 < r < 5.0:
                b = min(int(angles_deg[i] * n_bins / 360.0), n_bins - 1)
                if r < binned[b]:
                    binned[b] = r
        return binned

    def _load_signatures(self):
        self._pr_stored = load_all_signatures(SIGNATURE_DIR)
        if self._pr_stored:
            self._pr_n_bins = next(iter(self._pr_stored.values())).n_bins
            self.get_logger().info(
                f'PR: {len(self._pr_stored)} Signaturen geladen: '
                f'{sorted(self._pr_stored.keys())}  (n_bins={self._pr_n_bins})')
        else:
            self.get_logger().error(f'PR: Keine Signaturen in {SIGNATURE_DIR}!')

    # ─────────────────────────────────────────────────────────────────────────
    # Zustandsmaschine
    # ─────────────────────────────────────────────────────────────────────────

    def _loop(self):
        # ── INIT: Warte bis Lidar + IMU bereit ───────────────────────────
        if self.state == State.INIT:
            if self._imu_initialized and self._scan_ranges is not None:
                self.get_logger().info(
                    f'Sensoren bereit. Starte Place Recognition. '
                    f'IMU-Yaw: {math.degrees(self._imu_theta_raw):.1f}°')
                self.state = State.PR_SCAN
            return

        # ── PR_SCAN: Scans sammeln und PR-Erkennung durchfuehren ─────────
        if self.state == State.PR_SCAN:
            sig = self._scan_to_angle_signature()
            if sig is None:
                return
            self._pr_scan_buffer.append(sig)
            n = len(self._pr_scan_buffer)
            self.get_logger().info(f'PR: Scans gesammelt {n}/{PR_N_SCANS}...')
            if n < PR_N_SCANS:
                return

            # Mitteln (inf→nan→nanmean→inf)
            stacked = np.stack(self._pr_scan_buffer)
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                avg = np.nanmean(
                    np.where(np.isfinite(stacked), stacked, np.nan), axis=0)
            test_ranges = np.where(np.isfinite(avg), avg, np.inf)

            # Erkennung
            result = recognize(
                test_ranges, self._pr_stored, PR_THRESHOLD,
                self._pr_n_bins, top_k=PR_TOP_K)

            if not result['matched']:
                self.get_logger().warn(
                    f'PR: Kein Match (bester Dk={result["best_dk"]:.1f}, '
                    f'Kandidat={result["best_loc"]}). Neuer Versuch...')
                self._pr_scan_buffer.clear()
                return   # naechster Versuch beim naechsten Timer-Tick

            loc       = result['best_loc']
            angle_deg = result['angle_deg']
            self.get_logger().info(
                f'PR: Erkannt={loc}  Dk={result["best_dk"]:.0f}  '
                f'Abweichung={angle_deg:+.1f}°  IMU={math.degrees(self._imu_theta_raw):.1f}°')

            if loc not in FRONT_STOP:
                self.get_logger().error(f'PR: Unbekannte Location "{loc}" – abbruch.')
                self.state = State.DONE
                return

            self._detected_loc    = loc
            self._front_stop_dist = FRONT_STOP[loc]

            # ── Heading-Berechnung ──────────────────────────────────────
            east  = self._imu_east_ref
            west  = math.atan2(math.sin(east + math.pi),     math.cos(east + math.pi))
            north = math.atan2(math.sin(east + math.pi / 2), math.cos(east + math.pi / 2))
            south = math.atan2(math.sin(east - math.pi / 2), math.cos(east - math.pi / 2))

            # Trainings-Heading (Roboter-Ausrichtung bei der PR-Aufnahme)
            training_h = {'P2': east, 'P4': west, 'P6': west, 'P9': east}[loc]

            # Aktuelles Heading aus Cross-Korrelation ableiten
            angle_rad    = math.radians(angle_deg)
            corrected_h  = math.atan2(
                math.sin(training_h - angle_rad),
                math.cos(training_h - angle_rad))

            self.get_logger().info(
                f'PR: Training={math.degrees(training_h):.1f}°  '
                f'Abweichung={angle_deg:+.1f}°  '
                f'Korrigiert={math.degrees(corrected_h):.1f}°')

            # Ziel-Heading fuer naechsten Wegpunkt
            target_h = {'P2': south, 'P4': north, 'P6': south, 'P9': north}[loc]

            dtheta = math.degrees(self._angle_diff(target_h, corrected_h))

            # P2: empirischer +3°-Offset (Hardware-Tests ergaben konst. Untersch.)
            if loc == 'P2':
                dtheta += P2_SOUTH_TURN_OFFSET_DEG

            turn_sent = dtheta / TEENSY_SCALE
            self.get_logger().info(
                f'PR: Ziel-Heading={math.degrees(target_h):.1f}°  '
                f'dTheta={dtheta:.1f}°  Gesendet={turn_sent:.1f}°')

            time.sleep(0.3)
            self._feedback_received = False
            self._send_cmd(f'T:{turn_sent:.1f}')
            self.state = State.PR_TURN

        # ── PR_TURN: Warte auf Bestaetigung der Initialausrichtung ───────
        elif self.state == State.PR_TURN:
            if (self._feedback_received
                    and self._feedback_value is not None
                    and self._feedback_value.startswith('T:')):
                self._feedback_received = False
                self.get_logger().info(
                    f'Turn fertig ({self._detected_loc}). '
                    f'Starte Fahrt → Frontstopp={self._front_stop_dist:.3f}m')
                self._send_cmd(f'R:{SEGMENT_LEN:.4f}')
                self.state = State.PR_DRIVE

        # ── PR_DRIVE: Fahrt in Segmenten → 1. Wegpunkt ─────────────────
        elif self.state == State.PR_DRIVE:
            # Frontstopp-Pruefung (hoehere Prioritaet als Segment-Feedback)
            fd = self._front_wall_dist(half_angle_deg=15)
            if fd is not None:
                self.get_logger().info(
                    f'Drive1: Front={fd:.3f}m  Ziel={self._front_stop_dist:.3f}m')
            if fd is not None and fd <= self._front_stop_dist:
                self.get_logger().info(
                    f'Frontstopp WP1 bei {fd:.3f}m')
                self._send_cmd('S:0')
                time.sleep(0.3)

                if self._detected_loc == 'P2':
                    # MCL-Logik: Westdrehung anhand Suedwandwinkel (identisch DRIVING_WP3_SOUTH)
                    front_angle = self._front_wall_angle()
                    turn_deg    = -(90.0 - front_angle)   # keine /TEENSY_SCALE (wie MCL)
                    self.get_logger().info(
                        f'P2→WP3-West: front_angle={front_angle:.1f}°  T:{turn_deg:.1f}°')
                    self._feedback_received = False
                    self._send_cmd(f'T:{turn_deg:.1f}')
                    self.state = State.WP3_TURN
                else:
                    turn2_total = SECOND_TURN_DEG[self._detected_loc]
                    turn2_sent  = turn2_total / TEENSY_SCALE
                    self.get_logger().info(
                        f'WP2-Turn: {turn2_total:.1f}° → T:{turn2_sent:.1f}°  '
                        f'(Frontstopp WP2={SECOND_STOP[self._detected_loc]:.3f}m)')
                    self._feedback_received = False
                    self._send_cmd(f'T:{turn2_sent:.1f}')
                    self.state = State.SEG2_TURN
                return

            # Naechstes Segment nach D:-Feedback
            if (self._feedback_received
                    and self._feedback_value is not None
                    and self._feedback_value.startswith('D:')):
                self._feedback_received = False
                # Keine IMU-Korrektur: BNO055 nach grossem PR-Turn unkalibriert.
                # Geradeausfahrt korrekt, da Initialausrichtung per Cross-Korr. stimmt.
                self._send_cmd(f'R:{SEGMENT_LEN:.4f}')

        # ── WP3_TURN: P2 – Warte auf T: nach front_wall_angle()-Westdrehung ───
        elif self.state == State.WP3_TURN:
            if (self._feedback_received
                    and self._feedback_value is not None
                    and self._feedback_value.startswith('T:')):
                self._feedback_received = False
                time.sleep(0.3)
                turned = self._wall_correction_wp3()   # Feinausrichtung Suedwand
                if turned:
                    self.get_logger().info('WP3-Turn: Feinkorrektur noetig → WP3_ALIGN')
                    self.state = State.WP3_ALIGN
                else:
                    self.get_logger().info('WP3-Turn: Ausrichtung OK → starte WP3-Westfahrt')
                    self._feedback_received = False
                    self._send_cmd(f'R:{SEGMENT_LEN:.4f}')
                    self.state = State.WP3_WEST

        # ── WP3_ALIGN: P2 – Warte auf T: nach Wandkorrektur-Feinausrichtung ──
        elif self.state == State.WP3_ALIGN:
            if (self._feedback_received
                    and self._feedback_value is not None
                    and self._feedback_value.startswith('T:')):
                self._feedback_received = False
                time.sleep(0.1)
                self.get_logger().info('WP3-Align: Feinkorrektur fertig → starte WP3-Westfahrt')
                self._send_cmd(f'R:{SEGMENT_LEN:.4f}')
                self.state = State.WP3_WEST

        # ── WP3_WEST: P2 – Westfahrt mit Wandkorrektur → WP3 (0.60m) ────────
        # Frontstopp jeden Tick; Wandkorrektur nach jedem D:-Segment (wie MCL).
        # Dann ohne Pause direkt weiter zu P4.
        elif self.state == State.WP3_WEST:
            fd = self._front_wall_dist(half_angle_deg=10)
            if fd is not None:
                self.get_logger().info(
                    f'WP3-West: Front={fd:.3f}m  Ziel={WP3_WEST_STOP:.3f}m')
            if fd is not None and fd <= WP3_WEST_STOP:
                self.get_logger().info(f'WP3 erreicht ({fd:.3f}m) → direkt weiter zu P4')
                self._send_cmd('S:0')
                time.sleep(0.3)
                self._feedback_received = False
                self._send_cmd(f'R:{SEGMENT_LEN:.4f}')
                self.state = State.P4_WEST
                return
            if (self._feedback_received
                    and self._feedback_value is not None
                    and self._feedback_value.startswith('D:')):
                self._feedback_received = False
                self._wall_correction_wp3()          # sendet T: wenn noetig
                self._send_cmd(f'R:{SEGMENT_LEN:.4f}')  # Teensy queued T:→R:

        # ── P4_WEST: P2 – Westfahrt mit Wandkorrektur → P4 (0.25m) ──────────
        elif self.state == State.P4_WEST:
            fd = self._front_wall_dist(half_angle_deg=10)
            if fd is not None:
                self.get_logger().info(
                    f'P4-West: Front={fd:.3f}m  Ziel={P4_WEST_STOP:.3f}m')
            if fd is not None and fd <= P4_WEST_STOP:
                self.get_logger().info(f'P4 erreicht ({fd:.3f}m) → DONE')
                self._send_cmd('S:0')
                self.state = State.DONE
                return
            if (self._feedback_received
                    and self._feedback_value is not None
                    and self._feedback_value.startswith('D:')):
                self._feedback_received = False
                self._wall_correction_wp3()
                self._send_cmd(f'R:{SEGMENT_LEN:.4f}')

        # ── SEG2_TURN: Warte auf T:-Feedback nach Drehung vor WP2 ───────
        elif self.state == State.SEG2_TURN:
            if (self._feedback_received
                    and self._feedback_value is not None
                    and self._feedback_value.startswith('T:')):
                self._feedback_received = False
                self.get_logger().info(
                    f'WP2-Turn fertig ({self._detected_loc}). '
                    f'Starte Fahrt → Frontstopp={SECOND_STOP[self._detected_loc]:.3f}m')
                self._send_cmd(f'R:{SEGMENT_LEN:.4f}')
                self.state = State.SEG2_DRIVE

        # ── SEG2_DRIVE: Fahrt in Segmenten → 2. Wegpunkt ───────────────
        elif self.state == State.SEG2_DRIVE:
            # Frontstopp erst bei D:-Feedback pruefen (verhindert Lidar-Artefakte
            # direkt nach 90°-Drehung – identisch zur WP6-Logik in navigator.py).
            if (self._feedback_received
                    and self._feedback_value is not None
                    and self._feedback_value.startswith('D:')):
                self._feedback_received = False
                fd = self._front_wall_dist(half_angle_deg=15)
                stop2 = SECOND_STOP[self._detected_loc]
                if fd is not None:
                    self.get_logger().info(
                        f'Drive2: Front={fd:.3f}m  Ziel={stop2:.3f}m')
                if fd is not None and fd <= stop2:
                    self.get_logger().info(
                        f'Frontstopp WP2 bei {fd:.3f}m → {self._detected_loc}: alle Wegpunkte erreicht!')
                    self._send_cmd('S:0')
                    self.state = State.DONE
                    return
                self._send_cmd(f'R:{SEGMENT_LEN:.4f}')

        # ── DONE ─────────────────────────────────────────────────────────
        elif self.state == State.DONE:
            pass   # Fertig – Node bleibt aktiv bis Ctrl-C

    # ─────────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ─────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PRNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()