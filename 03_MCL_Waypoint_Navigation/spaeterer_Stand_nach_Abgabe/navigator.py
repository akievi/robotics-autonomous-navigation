"""
Navigator-Node: Zustandsmaschine fuer Waypoint-Navigation mit MCL.
Zustaende: INIT -> ALIGN -> DRIVE_SEGMENT -> STOP_UPDATE -> CHECK_POSE -> CORRECT -> NEXT -> DONE
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String, Int32
import numpy as np
import yaml
import os
import time
import math
import csv
import pickle
from enum import Enum, auto

from mcl_nav.mcl import MCL


class State(Enum):
    INIT          = auto()
    ALIGN         = auto()
    WAIT_ALIGN    = auto()
    DRIVE_SEGMENT = auto()
    DRIVING       = auto()   # kontinuierliches Fahren mit Lidar-Regelung (P1->P2)
    WAIT_TURN     = auto()   # wartet auf T:-Feedback nach Wandkorrektur, dann startet R:
    WAIT_DRIVE    = auto()
    DRIVING_WP3_SOUTH = auto()  # P2->P3 Phase 1: nach Süden auf untere_wand zu
    WAIT_TURN_WP3     = auto()  # wartet auf T:-Feedback nach 90°-Drehung Süd, dann R:
    WAIT_ALIGN_WP3_SOUTH = auto()  # wartet auf T:-Feedback nach Feinkorrektur Süd, dann R:
    WAIT_TURN_WP3_WEST= auto()  # wartet auf T:-Feedback nach 90°-Drehung West, dann R:
    WAIT_ALIGN_WP3_WEST = auto()   # wartet auf T:-Feedback nach Feinkorrektur West, dann R:
    WAIT_TURN_WP4     = auto()  # wartet auf T:-Feedback nach WP4 90°-Drehung, dann NEXT
    DRIVING_WP6_DIRECT= auto()  # P5->P6: 90° links, dann direkt per Odometrie
    WAIT_TURN_WP6     = auto()  # wartet auf T:-Feedback nach 90°-Drehung, dann D:
    DRIVING_WP7_SOUTH = auto()  # P6->P7 Phase 1: nach Süden fahren bis Frontstopp
    WAIT_TURN_WP7_SOUTH  = auto()  # wartet auf T: nach 90°-Links-Drehung bei P6
    WAIT_ALIGN_WP7_SOUTH = auto()  # wartet auf T: nach Nordwand-Feinkorrektur
    WAIT_TURN_WP7_WEST   = auto()  # wartet auf T: nach 90°-Rechts-Drehung + Südwand-Korrektur
    WAIT_ALIGN_WP7_WEST  = auto()  # wartet auf T: nach Südwand-Feinkorrektur
    DRIVING_WP7_FINAL    = auto()  # P7: Westfahrt mit Lidar-Rückwandstopp
    DRIVING_WP8_WEST  = auto()   # P7->P8: Westfahrt, Südwand links, Frontstopp
    WAIT_TURN_WP8     = auto()   # wartet auf T: nach Wandkorrektur WP8
    ALIGN_WP8_HEADING      = auto()   # IMU-Ausrichtung bei P8 (vor 180°-Turn)
    WAIT_ALIGN_WP8_HEADING = auto()   # wartet auf T: nach WP8-Heading-Korrektur
    DRIVING_WP9_EAST  = auto()   # P8->P9: Ostfahrt, Südwand links, Rückwandstopp
    WAIT_TURN_WP9     = auto()   # wartet auf T: nach Wandkorrektur WP9
    WAIT_TURN_WP9_180 = auto()   # wartet auf T: nach 180°-Drehung bei P8
    DRIVING_WP10_NORTH = auto()  # P9->P10: +y Fahrt (physisch Süd), Frontstopp obere_wand
    WAIT_TURN_WP10     = auto()  # wartet auf T: nach 90°-Links-Drehung bei P9
    DRIVING_WP11_EAST  = auto()  # P10->P11: -x Fahrt (physisch Ost), Frontstopp linke_abschlusswand
    WAIT_TURN_WP11     = auto()  # wartet auf T: nach 90°-Links-Drehung bei P10
    DRIVING_WP3_WEST  = auto()  # P2->P3 Phase 2: nach Westen entlang untere_wand
    DRIVING_WP4_WEST  = auto()  # P3->P4: weiter nach Westen, dann 90° rechts
    STOP_UPDATE   = auto()
    CHECK_POSE    = auto()
    CORRECT       = auto()
    WAIT_CORRECT  = auto()
    NEXT          = auto()
    DONE          = auto()


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        # Konfiguration laden
        from ament_index_python.packages import get_package_share_directory
        cfg_path = os.path.join(
            get_package_share_directory('mcl_nav'), 'config', 'map_config.yaml')
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)

        self.robot_cfg = cfg['robot']
        self.nav_cfg   = cfg['navigation']
        self.mcl_cfg   = cfg['mcl']
        self.waypoints = cfg['waypoints']
        walls_raw = cfg.get('walls', [])
        walls = [((w['x1'], w['y1']), (w['x2'], w['y2'])) for w in walls_raw]

        self.mcl = MCL(self.mcl_cfg, walls)

        # Startpose
        wp0 = self.waypoints[0]
        self.mcl.init_pose(wp0['x'], wp0['y'], np.deg2rad(wp0['theta']))
        self.pose = (wp0['x'], wp0['y'], np.deg2rad(wp0['theta']))  # (x, y, theta_rad)

        self._imu_initialized = False
        self._imu_theta = np.deg2rad(wp0['theta'])
        self._imu_theta_raw = None  # letzter Quaternion-Yaw

        # Odometrie-Akkumulator
        self._odom_dist  = 0.0
        self._odom_theta = 0.0

        # Letzter Lidar-Scan
        self._scan_ranges = None
        self._scan_angles = None

        # Zustandsmaschine
        self.state = State.INIT
        self._wp_index = 1        # naechster Waypoint (0 = Start)
        self._segment_driven = 0.0
        self._correction_attempts = 0
        self._feedback_received = False
        self._feedback_value = None
        self._align_done = False
        self._pending_drive_dist = 0.0  # nach WAIT_TURN: diese Distanz fahren
        self._cmd_sent_time = None      # Zeitstempel letzter _send_cmd für Timeout

        # Logging
        log_dir = '/home/pi/ros2_ws/logs'
        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self._csv = open(f'{log_dir}/run_{ts}.csv', 'w', newline='')
        self._csv_writer = csv.writer(self._csv)
        self._csv_writer.writerow(['time', 'state', 'wp', 'true_x', 'true_y',
                                   'est_x', 'est_y', 'est_theta', 'n_eff'])
        self._particle_log = []
        self._log_path = f'{log_dir}/particles_{ts}.pkl'

        # Publisher
        self._cmd_pub  = self.create_publisher(String, 'teensy_cmd', 10)
        self._stop_pub = self.create_publisher(Int32,  'micro_ros_fabinal_subscriber', 10)

        # Subscriber
        self.create_subscription(
            LaserScan, '/scan',
            self._scan_cb,
            QoSPresetProfiles.SENSOR_DATA.value)

        self.create_subscription(
            Imu, '/imu/data',
            self._imu_cb, 10)

        self.create_subscription(
            String, 'odom_feedback',
            self._feedback_cb, 10)

        # Wandregler-Parameter
        self._wall_target    = self.nav_cfg.get('wall_target_dist', 0.27)
        self._wall_kp        = self.nav_cfg.get('wall_kp', 80.0)
        self._wall_deadband  = self.nav_cfg.get('wall_deadband', 0.02)   # ±2cm → C:0
        self._wall_realign_threshold = self.nav_cfg.get('wall_realign_threshold', 0.05)  # 5cm
        self._wall_active          = False
        self._wall_prev_dist       = None
        self._wall_was_large_error = False
        self._wall_last_error_sign = 0
        self._front_stop_dist      = None
        self._imu_heading_ref      = None  # IMU-Yaw beim Start des Segments
        self._imu_east_ref         = math.radians(self.nav_cfg.get('imu_east_ref_deg', 0.0))
        self._imu_glitch_detected  = False  # True wenn BNO055-Sprung während WP8 erkannt
        self._imu_glitch_ref       = None   # letzter guter IMU-Wert vor dem Glitch
        self._align_wp8_attempts   = 0     # Zähler für P8-Heading-Korrekturen
        self._wp3_phase            = None  # 'south' oder 'west' für WP3-Navigation
        self._wp3_driving          = False  # True wenn gerade ein Segment läuft

        # Hauptschleife 10 Hz
        self.create_timer(0.1, self._loop)
        # Wandregler 20 Hz (schneller als Hauptschleife)
        self.create_timer(0.05, self._wall_control_loop)

        self.get_logger().info('Navigator gestartet. Warte auf ersten Lidar-Scan...')

    # ── Sensor-Callbacks ────────────────────────────────────────────

    def _scan_cb(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = (np.arange(len(ranges)) * msg.angle_increment + msg.angle_min)
        # Ungueltige Werte maskieren
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges[~valid] = np.inf
        self._scan_ranges = ranges
        self._scan_angles = angles

    def _imu_cb(self, msg):
        # Quaternion -> Yaw (nur Z-Rotation)
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self._imu_theta_raw = yaw
        if not self._imu_initialized:
            self._imu_theta = yaw
            self._imu_initialized = True
            self.get_logger().info(f'IMU initialisiert: raw={math.degrees(yaw):.1f}°  (imu_east_ref_deg={math.degrees(self._imu_east_ref):.1f}°)')

    def _feedback_cb(self, msg):
        self._feedback_value = msg.data
        # R:ok und C:ok sind keine echten Fertig-Signale
        if msg.data.startswith('R:') or msg.data.startswith('C:'):
            return
        self._feedback_received = True
        self.get_logger().info(f'Feedback: {msg.data}')

        data = msg.data
        if data.startswith('D:'):
            val = float(data[2:])
            self._odom_dist += val
            self._segment_driven += val
        elif data.startswith('T:'):
            actual_rad = np.deg2rad(float(data[2:]))
            self._odom_theta += actual_rad
            self._imu_theta  += actual_rad
            self._imu_theta   = (self._imu_theta + math.pi) % (2 * math.pi) - math.pi

    # ── Hilfsfunktionen ─────────────────────────────────────────────

    def _left_wall_dist(self):
        """Abstand zur linken Wand aus Lidar (Winkel +60°..+120° relativ zum Roboter)."""
        if self._scan_ranges is None:
            return None
        angles = self._scan_angles
        ranges = self._scan_ranges
        mask = (angles > np.deg2rad(60)) & (angles < np.deg2rad(120)) & \
               np.isfinite(ranges) & (ranges < 3.0)
        if np.sum(mask) < 3:
            return None
        return float(np.median(ranges[mask]))

    def _front_wall_dist(self, half_angle_deg=35):
        """Minimaler Abstand zur Vorderwand aus Lidar."""
        if self._scan_ranges is None:
            return None
        angles = self._scan_angles
        ranges = self._scan_ranges
        mask = (angles > np.deg2rad(-half_angle_deg)) & (angles < np.deg2rad(half_angle_deg)) & \
               np.isfinite(ranges) & (ranges < 3.0)
        if np.sum(mask) < 3:
            return None
        return float(np.min(ranges[mask]))

    def _rear_wall_dist(self, half_angle_deg=20):
        """Minimaler Abstand zur Rückwand (180°) aus Lidar."""
        if self._scan_ranges is None:
            return None
        angles = self._scan_angles
        ranges = self._scan_ranges
        mask = (np.abs(np.abs(angles) - np.pi) < np.deg2rad(half_angle_deg)) & \
               np.isfinite(ranges) & (ranges < 3.0)
        if np.sum(mask) < 3:
            return None
        return float(np.min(ranges[mask]))

    def _front_wall_angle(self, half_angle_deg=25):
        """Winkel des naechsten Punktes zur vorderen Wand. Positiv = Wand leicht rechts von 0°."""
        if self._scan_ranges is None:
            self.get_logger().warn('_front_wall_angle: kein Scan verfügbar, Fallback 0°')
            return 0.0
        angles = self._scan_angles
        ranges = self._scan_ranges
        mask = (angles > np.deg2rad(-half_angle_deg)) & (angles < np.deg2rad(half_angle_deg)) & \
               np.isfinite(ranges) & (ranges < 3.0)
        n = int(np.sum(mask))
        if n < 3:
            self.get_logger().warn(f'_front_wall_angle: zu wenige Punkte ({n}), Fallback 0°')
            return 0.0
        masked_ranges = ranges.copy()
        masked_ranges[~mask] = np.inf
        idx = int(np.argmin(masked_ranges))
        angle = float(np.degrees(angles[idx]))
        self.get_logger().info(f'_front_wall_angle: {angle:.1f}° (naechster Punkt {ranges[idx]:.3f}m, {n} Punkte im Fenster)')
        return angle

    def _front_wall_angle_regression(self, half_angle_deg=45):
        """Wandnormale per linearer Regression — für weit entfernte, freie Wände (P5, P6)."""
        if self._scan_ranges is None:
            self.get_logger().warn('_front_wall_angle_regression: kein Scan verfügbar, Fallback 0°')
            return 0.0
        angles = self._scan_angles
        ranges = self._scan_ranges
        mask = (angles > np.deg2rad(-half_angle_deg)) & (angles < np.deg2rad(half_angle_deg)) & \
               np.isfinite(ranges) & (ranges < 3.0)
        n = int(np.sum(mask))
        if n < 5:
            self.get_logger().warn(f'_front_wall_angle_regression: zu wenige Punkte ({n}), Fallback 0°')
            return 0.0
        a = angles[mask]
        r = ranges[mask]
        xs = r * np.cos(a)
        ys = r * np.sin(a)
        xm, ym = np.mean(xs), np.mean(ys)
        dx = xs - xm
        m = float(np.dot(dx, ys - ym) / (np.dot(dx, dx) + 1e-9))
        wall_angle_deg = float(np.degrees(np.arctan(m)))
        self.get_logger().info(
            f'_front_wall_angle_regression: {wall_angle_deg:.1f}° ({n} Punkte, dist={float(np.mean(r)):.3f}m)')
        return wall_angle_deg

    def _wall_control_loop(self):
        """Nur Logging während der Fahrt — Korrektur passiert per Turn nach jedem Segment."""
        if not self._wall_active:
            return
        min_dist, min_angle = self._left_wall_min()
        if min_dist is None:
            return
        error = min_dist - self._wall_target
        self.get_logger().info(
            f'Wand: {min_dist:.3f}m  angle={min_angle:.1f}°  err={error*100:.1f}cm')

    def _left_wall_min(self, half_angle_deg=45):
        """Minimaler Abstand und Winkel zur linken Wand im Bereich (90-half)°-(90+half)°."""
        if self._scan_ranges is None:
            return None, None
        angles = self._scan_angles
        ranges = self._scan_ranges
        mask = (angles > np.deg2rad(90 - half_angle_deg)) & (angles < np.deg2rad(90 + half_angle_deg)) & \
               np.isfinite(ranges) & (ranges < 3.0)
        if np.sum(mask) < 3:
            return None, None
        idx = np.argmin(ranges[mask])
        min_dist = float(ranges[mask][idx])
        min_angle_deg = float(np.degrees(angles[mask][idx]))
        return min_dist, min_angle_deg

    def _south_wall_min(self):
        """Minimaler Abstand zur südlichen Wand im Bereich -45° bis -135° (links wenn nach Westen)."""
        if self._scan_ranges is None:
            return None, None
        angles = self._scan_angles
        ranges = self._scan_ranges
        mask = (angles > np.deg2rad(-135)) & (angles < np.deg2rad(-45)) & \
               np.isfinite(ranges) & (ranges < 3.0)
        if np.sum(mask) < 3:
            return None, None
        idx = np.argmin(ranges[mask])
        min_dist = float(ranges[mask][idx])
        min_angle_deg = float(np.degrees(angles[mask][idx]))
        return min_dist, min_angle_deg

    def _wall_angle_correction_wp3_west(self):
        """Wandkorrektur für WP3/WP4-West: untere_wand links (75°-105°). Gibt True zurück wenn T: gesendet."""
        min_dist, _ = self._left_wall_min(half_angle_deg=15)
        if min_dist is None:
            return False

        wall_target = self.nav_cfg.get('wp3_wall_target_dist', self._wall_target)
        dist_err = min_dist - wall_target  # positiv = zu weit, negativ = zu nah
        gain = self.nav_cfg.get('wp3_wall_angle_gain', self.nav_cfg.get('wall_angle_gain', 15.0))
        turn_deg = dist_err * gain
        turn_deg = max(-15.0, min(15.0, turn_deg))

        self.get_logger().info(
            f'WP3-Wandkorrektur: dist={min_dist:.3f}m  err={dist_err*100:.1f}cm  Drehe={turn_deg:.1f}°')
        if abs(turn_deg) > 0.5:
            self._send_cmd(f'T:{turn_deg:.1f}')
            return True
        self._imu_heading_ref = self._imu_theta_raw
        return False

    def _wall_angle_correction(self):
        """Absoluter Zielwinkel proportional zum Abstandsfehler.
        Gibt True zurück wenn ein T:-Kommando gesendet wurde, sonst False."""
        min_dist, _ = self._left_wall_min()
        if min_dist is None:
            return False

        dist_err = min_dist - self._wall_target  # positiv = zu weit, negativ = zu nah
        seg = self.nav_cfg['segment_length']
        target_deg = math.degrees(math.atan2(dist_err, seg)) * 0.5
        target_deg = max(-5.0, min(5.0, target_deg))

        if self._imu_heading_ref is not None and self._imu_theta_raw is not None:
            heading_deg = math.degrees(self._angle_diff(self._imu_theta_raw, self._imu_heading_ref))
        else:
            heading_deg = math.degrees(self.pose[2])

        if abs(heading_deg) > 15.0:
            self.get_logger().warn(f'IMU-Drift erkannt ({heading_deg:.1f}°) — Referenz neu gesetzt')
            self._imu_heading_ref = self._imu_theta_raw
            return False

        turn_deg = target_deg - heading_deg
        turn_deg = max(-5.0, min(5.0, turn_deg))

        self.get_logger().info(
            f'Wandkorrektur: dist={min_dist:.3f}m  err={dist_err*100:.1f}cm  '
            f'target={target_deg:.1f}°  heading={heading_deg:.1f}°  Drehe={turn_deg:.1f}°')
        if abs(turn_deg) > 1.0:
            self._send_cmd(f'T:{turn_deg:.1f}')
            return True
        self._imu_heading_ref = self._imu_theta_raw
        return False

    def _wall_angle_correction_strong(self, negate=False, use_south_wall=False, max_deg=10.0):
        """Wandkorrektur, stärker als _wall_angle_correction: kein *0.5-Faktor, Limit max_deg°.
        use_south_wall=True: misst rechte Seite (-90°) = Südwand direkt (für P9-Ostfahrt).
        negate=True: Vorzeichen invertieren. max_deg: Begrenzung in Grad (Standard 10°)."""
        if use_south_wall:
            min_dist, _ = self._south_wall_min()
        else:
            min_dist, _ = self._left_wall_min()
        if min_dist is None:
            return False

        dist_err = min_dist - self._wall_target
        seg = self.nav_cfg['segment_length']
        target_deg = math.degrees(math.atan2(dist_err, seg))
        if negate:
            target_deg = -target_deg  # nur Distanzkomponente invertieren (rechte Wand)
        target_deg = max(-max_deg, min(max_deg, target_deg))

        if self._imu_heading_ref is not None and self._imu_theta_raw is not None:
            heading_deg = math.degrees(self._angle_diff(self._imu_theta_raw, self._imu_heading_ref))
        else:
            heading_deg = math.degrees(self.pose[2])

        if abs(heading_deg) > 20.0:
            self.get_logger().warn(f'IMU-Drift erkannt ({heading_deg:.1f}°) — Referenz neu gesetzt')
            if not self._imu_glitch_detected:
                self._imu_glitch_detected = True
                self._imu_glitch_ref = self._imu_heading_ref  # letzter guter Wert vor dem Sprung
            self._imu_heading_ref = self._imu_theta_raw
            return False

        turn_deg = target_deg - heading_deg
        turn_deg = max(-max_deg, min(max_deg, turn_deg))

        self.get_logger().info(
            f'Wandkorrektur: dist={min_dist:.3f}m  err={dist_err*100:.1f}cm  '
            f'target={target_deg:.1f}°  heading={heading_deg:.1f}°  Drehe={turn_deg:.1f}°')
        if abs(turn_deg) > 1.0:
            self._send_cmd(f'T:{turn_deg:.1f}')
            return True
        self._imu_heading_ref = self._imu_theta_raw
        return False

    def _send_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)
        self._feedback_received = False
        self._cmd_sent_time = time.time()
        self.get_logger().info(f'CMD: {cmd}')

    def _stop(self):
        msg = Int32()
        msg.data = 500
        self._stop_pub.publish(msg)

    def _angle_diff(self, a, b):
        """Vorzeichenbehaftete Differenz a-b in [-pi, pi]."""
        d = a - b
        return (d + math.pi) % (2 * math.pi) - math.pi

    def _mcl_update(self, delta_dist, delta_theta):
        """Motion predict + Lidar + IMU-Update + Resample + Pose schaetzen."""
        self.mcl.motion_predict(delta_dist, delta_theta)

        if self._scan_ranges is not None:
            ox = self.robot_cfg['lidar_offset_x']
            oy = self.robot_cfg['lidar_offset_y']
            self.mcl.measurement_update(
                self._scan_ranges, self._scan_angles, ox, oy)

        # IMU deaktiviert — driftet zu stark (mag=0)
        # if self._imu_theta_raw is not None:
        #     self.mcl.imu_update(...)

        self.mcl.resample()

        x, y, theta = self.mcl.estimate_pose()
        self.pose = (x, y, theta)

        neff = self.mcl.effective_particle_count()
        self._log(x, y, theta, neff)
        self.get_logger().info(
            f'MCL: pos=({x:.3f},{y:.3f}) theta={math.degrees(theta):.1f}° '
            f'IMU={math.degrees(self._imu_theta_raw):.1f}° N_eff={neff:.0f}')
        return x, y, theta

    def _log(self, x, y, theta, neff):
        wp = self._wp_index
        self._csv_writer.writerow([
            time.time(), self.state.name, wp, x, y, theta, neff])
        self._csv.flush()
        # Partikel-Snapshot alle 5 Updates
        if len(self._particle_log) % 5 == 0:
            self._particle_log.append(self.mcl.particles.copy())

    def _dist_to_goal(self):
        wp = self.waypoints[self._wp_index]
        return math.hypot(wp['x'] - self.pose[0], wp['y'] - self.pose[1])

    # ── Hauptschleife ───────────────────────────────────────────────

    def _loop(self):
        if self.state == State.INIT:
            if self._scan_ranges is not None and self._imu_initialized:
                self.get_logger().info(
                    f'Sensoren bereit. Starte Navigation. IMU-Yaw: {math.degrees(self._imu_theta):.1f}°')
                self.state = State.ALIGN

        elif self.state == State.ALIGN:
            wp = self.waypoints[self._wp_index]
            # Fahrtrichtung zum nächsten Waypoint geometrisch berechnen
            dx = wp['x'] - self.pose[0]
            dy = wp['y'] - self.pose[1]
            target_theta = math.atan2(dy, dx)
            # Aktuelles Heading: IMU zuverlässiger als MCL bei N_eff=800
            if self._imu_theta_raw is not None:
                current_theta = self._imu_theta_raw
            else:
                current_theta = self.pose[2]
            dtheta = self._angle_diff(target_theta, current_theta)

            # T: immer senden (auch bei <5°), damit T:-Feedback bestätigt dass Teensy empfangsbereit ist
            # bevor das erste R: geschickt wird
            self.get_logger().info(f'P{wp["id"]}: Ausrichten {math.degrees(dtheta):.1f}°')
            self._odom_dist = 0.0
            self._odom_theta = 0.0
            self._send_cmd(f'T:{math.degrees(dtheta):.2f}')
            self.state = State.WAIT_ALIGN

        elif self.state == State.WAIT_ALIGN:
            if self._feedback_received:
                # MCL nach Drehung updaten
                self._mcl_update(0.0, self._odom_theta)
                self._odom_theta = 0.0
                self.state = State.DRIVE_SEGMENT
            elif self._cmd_sent_time is not None and (time.time() - self._cmd_sent_time) > 5.0:
                self.get_logger().warn('WAIT_ALIGN: kein T:-Feedback nach 5s — Teensy noch nicht bereit, sende erneut')
                self.state = State.ALIGN

        elif self.state == State.DRIVE_SEGMENT:
            wp = self.waypoints[self._wp_index]
            wp0 = self.waypoints[self._wp_index - 1] if self._wp_index > 0 else wp
            total_dist = math.hypot(wp['x'] - wp0['x'], wp['y'] - wp0['y'])
            remaining = max(0.0, total_dist - self._segment_driven)
            seg = self.nav_cfg['segment_length']
            dist = min(remaining, seg)
            if remaining > 0.05:
                self._odom_dist = 0.0
                wall_segments = self.nav_cfg.get('wall_follow_segments', [])
                self._wall_active = wp['id'] in wall_segments
                self._imu_heading_ref = self._imu_theta_raw
                self._front_stop_dist = self.nav_cfg.get(f'front_stop_wp{wp["id"]}', None)
                self._feedback_received = False
                self._send_cmd(f'R:{dist:.4f}')
                self.state = State.DRIVING
            else:
                self.state = State.STOP_UPDATE

        elif self.state == State.DRIVING:
            # Lidar-basierter Frühstopp
            min_driven = self.nav_cfg.get('front_stop_min_driven', 0.3)
            if self._front_stop_dist is not None and self._segment_driven > min_driven:
                fd = self._front_wall_dist(half_angle_deg=15)
                if fd is not None:
                    self.get_logger().info(f'Front: {fd:.3f}m  Ziel: {self._front_stop_dist:.3f}m')
                if fd is not None and fd <= self._front_stop_dist:
                    self.get_logger().info(f'Frontstopp: {fd:.3f}m')
                    self._send_cmd('S:0')
                    self._wall_active = False
                    time.sleep(0.3)
                    self._odom_dist = 0.0
                    self.state = State.NEXT
                    return

            # Warte auf D:-Feedback vom Teensy (Segment fertig) — T:-Feedback ignorieren
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('D:'):
                self._wall_active = False
                time.sleep(0.15)
                actual = self._odom_dist
                self._odom_dist = 0.0
                self._mcl_update(actual, 0.0)
                wp = self.waypoints[self._wp_index]
                wp0 = self.waypoints[self._wp_index - 1] if self._wp_index > 0 else wp
                total_dist = math.hypot(wp['x'] - wp0['x'], wp['y'] - wp0['y'])
                remaining = max(0.0, total_dist - self._segment_driven)
                self.get_logger().info(
                    f'Segment: gefahren={self._segment_driven:.2f}m  remaining={remaining:.2f}m')
                if remaining < 0.05:
                    self.state = State.STOP_UPDATE
                else:
                    # Wandkorrektur, dann nächstes Segment
                    turn_sent = self._wall_angle_correction()
                    wp0 = self.waypoints[self._wp_index - 1] if self._wp_index > 0 else wp
                    total_dist = math.hypot(wp['x'] - wp0['x'], wp['y'] - wp0['y'])
                    remaining = max(0.0, total_dist - self._segment_driven)
                    seg = self.nav_cfg['segment_length']
                    dist = min(remaining, seg)
                    if remaining > 0.05:
                        self._odom_dist = 0.0
                        wall_segments = self.nav_cfg.get('wall_follow_segments', [])
                        self._wall_active = wp['id'] in wall_segments
                        if turn_sent:
                            # Warte auf T:-Feedback bevor R: gesendet wird
                            self._pending_drive_dist = dist
                            self.state = State.WAIT_TURN
                        else:
                            self._imu_heading_ref = self._imu_theta_raw
                            self._feedback_received = False
                            self._send_cmd(f'R:{dist:.4f}')
                            # Bleibe in DRIVING
                    else:
                        self.state = State.STOP_UPDATE

        elif self.state == State.WAIT_DRIVE:
            # Nicht mehr genutzt — DRIVING ersetzt diesen State
            self.state = State.DRIVE_SEGMENT

        elif self.state == State.WAIT_TURN:
            # Warte auf T:-Feedback nach Wandkorrektur, dann sende R:
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                self._send_cmd(f'R:{self._pending_drive_dist:.4f}')
                self.state = State.DRIVING

        elif self.state == State.WAIT_TURN_WP3:
            # Warte auf T:-Feedback nach 90°-Drehung nach Süden, dann starte Südfahrt
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self.get_logger().info('WP3-Süd: Drehung nach Süden fertig, starte Südfahrt')
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                self._send_cmd(f'R:{self._pending_drive_dist:.4f}')
                self.state = State.DRIVING_WP3_SOUTH

        elif self.state == State.WAIT_ALIGN_WP3_SOUTH:
            # Nicht genutzt (Südwand liegt vorne, nicht links — Korrektur nicht möglich)
            pass

        elif self.state == State.WAIT_TURN_WP3_WEST:
            # Warte auf T:-Feedback nach 90°-Drehung nach Westen, dann Feinkorrektur per Südwand
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self.get_logger().info('WP3-West: Drehung nach Westen fertig, prüfe Ausrichtung zur Südwand')
                self._feedback_received = False
                time.sleep(0.3)
                turned = self._wall_angle_correction_wp3_west()
                if turned:
                    self.state = State.WAIT_ALIGN_WP3_WEST
                else:
                    self.get_logger().info('WP3-West: Ausrichtung OK, starte Westfahrt')
                    self._imu_heading_ref = self._imu_theta_raw
                    self._wall_active = True
                    self._send_cmd(f'R:{self._pending_drive_dist:.4f}')
                    self.state = State.DRIVING_WP3_WEST

        elif self.state == State.WAIT_ALIGN_WP3_WEST:
            # Warte auf T:-Feedback der Feinkorrektur, dann starte Westfahrt
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self.get_logger().info('WP3-West: Feinkorrektur fertig, starte Westfahrt')
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                self._wall_active = True
                self._send_cmd(f'R:{self._pending_drive_dist:.4f}')
                self.state = State.DRIVING_WP3_WEST

        elif self.state == State.WAIT_TURN_WP4:
            # Warte auf T:-Feedback nach WP4 90°-Drehung, dann NEXT
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self.get_logger().info('WP4-West: Drehung fertig, gehe zu NEXT')
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                self.state = State.NEXT

        elif self.state == State.WAIT_TURN_WP6:
            # Warte auf T:-Feedback nach 90°-Drehung links, dann erste R: starten
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self.get_logger().info('WP6: Drehung fertig, fahre Richtung P6 mit Lidar-Frontstopp')
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                time.sleep(0.3)
                seg = self.nav_cfg.get('wp6_segment_length', 0.12)
                self._send_cmd(f'R:{seg:.4f}')
                self.state = State.DRIVING_WP6_DIRECT

        elif self.state == State.WAIT_TURN_WP7_SOUTH:
            # Warte auf T: nach 90°-Links-Drehung bei P6, dann Südfahrt starten
            # KEINE Wandkorrektur hier: _left_wall_min() liest bei Südfahrt die Ostseite (+x),
            # nicht die Nordwand — falsche Messung führt zu bis zu +15° Linkskorrektur → trenner_unten
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self.get_logger().info('WP7-Süd: Drehung nach Süden fertig, starte Südfahrt (kein Wandabgleich)')
                self._feedback_received = False
                self._imu_heading_ref = self._imu_theta_raw
                self._send_cmd('R:0.3000')
                self._wp3_driving = True
                self.state = State.DRIVING_WP7_SOUTH

        elif self.state == State.WAIT_ALIGN_WP7_SOUTH:
            # Warte auf T: nach Nordwand-Feinkorrektur, dann starte Südfahrt
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self.get_logger().info('WP7-Süd: Feinkorrektur fertig, starte Südfahrt')
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                self._send_cmd('R:0.3000')
                self._wp3_driving = True
                self.state = State.DRIVING_WP7_SOUTH

        elif self.state == State.DRIVING_WP7_SOUTH:
            # Nach Süden fahren bis Frontstopp (Südwand-Abstand)
            # half_angle_deg=15: enger Kegel vermeidet Erkennung von trenner_unten im Winkel
            front_stop = self.nav_cfg.get('wp7_south_stop_dist', 0.30)
            fd = self._front_wall_dist(half_angle_deg=15)
            if fd is not None:
                self.get_logger().info(f'WP7-Süd: Front={fd:.3f}m  Ziel={front_stop:.3f}m')
            if fd is not None and fd <= front_stop:
                self.get_logger().info(f'WP7-Süd: Stopp bei {fd:.3f}m, drehe 90° nach Westen')
                self._send_cmd('S:0')
                time.sleep(0.3)
                self.get_logger().info('WP7-Süd: Drehe -87.4° nach Westen (1.03-Vorkorrektur → actual -90°)')
                self._feedback_received = False
                self._send_cmd('T:-87.4')
                self.state = State.WAIT_TURN_WP7_WEST
            elif self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('D:'):
                self._feedback_received = False
                self._send_cmd('R:0.3000')

        elif self.state == State.WAIT_TURN_WP7_WEST:
            # Warte auf T: nach 90°-Rechts-Drehung, dann Südwand-Feinkorrektur
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self.get_logger().info('WP7-West: Drehung nach Westen fertig, prüfe Ausrichtung zur Südwand')
                self._feedback_received = False
                time.sleep(0.3)
                min_dist, _ = self._left_wall_min(half_angle_deg=15)
                if min_dist is not None:
                    wall_target = self.nav_cfg.get('wp7_south_wall_target_dist', 0.27)
                    dist_err = min_dist - wall_target
                    gain = self.nav_cfg.get('wall_angle_gain', 50.0)
                    turn_deg = dist_err * gain
                    turn_deg = max(-15.0, min(15.0, turn_deg))
                    self.get_logger().info(
                        f'WP7-West: Südwand dist={min_dist:.3f}m  err={dist_err*100:.1f}cm  Drehe={turn_deg:.1f}°')
                    if abs(turn_deg) > 1.0:
                        self._send_cmd(f'T:{turn_deg:.1f}')
                        self.state = State.WAIT_ALIGN_WP7_WEST
                        return
                self.get_logger().info('WP7-West: Ausrichtung OK, starte Westfahrt mit Lidar-Rückwandstopp')
                self._feedback_received = False
                self._send_cmd('R:0.3000')
                self.state = State.DRIVING_WP7_FINAL

        elif self.state == State.WAIT_ALIGN_WP7_WEST:
            # Warte auf T: nach Südwand-Feinkorrektur, dann mit R: Westfahrt starten
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self.get_logger().info('WP7-West: Feinkorrektur fertig, starte Westfahrt mit Lidar-Rückwandstopp')
                self._feedback_received = False
                self._send_cmd('R:0.3000')
                self.state = State.DRIVING_WP7_FINAL

        elif self.state == State.DRIVING_WP7_FINAL:
            # Westfahrt: Rückwand (trenner_unten bei x=1.62) wächst mit Fahrt → Stopp bei Zielabstand
            rear_stop = self.nav_cfg.get('wp7_rear_stop_dist', 0.34)
            rd = self._rear_wall_dist()
            if rd is not None:
                self.get_logger().info(f'WP7-Final: Rückwand={rd:.3f}m  Ziel={rear_stop:.3f}m')
            if rd is not None and rd >= rear_stop:
                self.get_logger().info(f'WP7: Stopp bei Rückwand {rd:.3f}m, P7 erreicht')
                self._send_cmd('S:0')
                self._feedback_received = False
                self.state = State.NEXT
            elif self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('D:'):
                self._feedback_received = False
                self._send_cmd('R:0.3000')

        elif self.state == State.DRIVING_WP8_WEST:
            # P7->P8: Westfahrt, Südwand links (+90° Lidar), Frontstopp linke_abschlusswand
            # max_deg=7° (reduziert von 13°): ruhigere Fahrt, weniger Pendelkorrektur
            front_stop = self.nav_cfg.get('front_stop_wp8', 0.28)
            self._wall_target = self.nav_cfg.get('wp8_south_wall_target_dist', 0.27)
            seg = self.nav_cfg.get('wp8_segment_length', 0.12)
            fd = self._front_wall_dist(half_angle_deg=15)
            if fd is not None:
                self.get_logger().info(f'WP8-West: Front={fd:.3f}m  Ziel={front_stop:.3f}m')
            if fd is not None and fd <= front_stop:
                self.get_logger().info(f'WP8: Frontstopp bei {fd:.3f}m, P8 erreicht')
                self._send_cmd('S:0')
                self._feedback_received = False
                self.state = State.NEXT
            elif self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('D:'):
                self._feedback_received = False
                turn_sent = self._wall_angle_correction_strong(max_deg=7.0)
                if turn_sent:
                    self.state = State.WAIT_TURN_WP8
                else:
                    self._imu_heading_ref = self._imu_theta_raw
                    self._send_cmd(f'R:{seg:.4f}')

        elif self.state == State.WAIT_TURN_WP8:
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                seg = self.nav_cfg.get('wp8_segment_length', 0.12)
                self._send_cmd(f'R:{seg:.4f}')
                self.state = State.DRIVING_WP8_WEST

        elif self.state == State.ALIGN_WP8_HEADING:
            # IMU-basierte Heading-Korrektur bei P8 vor 180°-Turn:
            # Ziel = gleicher Winkel wie beim Start der Westfahrt (_wp8_entry_heading).
            # Korrigiert akkumulierten Heading-Fehler durch Pendelkorrekturen während P7->P8.
            max_attempts = 3
            threshold_deg = 2.0
            if self._wp8_entry_heading is None or self._align_wp8_attempts >= max_attempts:
                self.get_logger().info(
                    f'WP8-Ausricht: {"kein Ref" if self._wp8_entry_heading is None else "Max Versuche"} → 180°-Turn')
                self._send_cmd('T:-180.0')
                self.state = State.WAIT_TURN_WP9_180
                return
            if self._imu_theta_raw is None:
                return
            drift_deg = math.degrees(self._angle_diff(self._imu_theta_raw, self._wp8_entry_heading))
            correction = max(-15.0, min(15.0, -drift_deg))
            self.get_logger().info(
                f'WP8-Ausricht #{self._align_wp8_attempts + 1}: Drift={drift_deg:.1f}°  Korrektur={correction:.1f}°')
            self._align_wp8_attempts += 1
            if abs(drift_deg) >= threshold_deg:
                self._send_cmd(f'T:{correction:.1f}')
                self.state = State.WAIT_ALIGN_WP8_HEADING
            else:
                self.get_logger().info('WP8-Ausricht: Heading OK → 180°-Turn')
                self._send_cmd('T:-180.0')
                self.state = State.WAIT_TURN_WP9_180

        elif self.state == State.WAIT_ALIGN_WP8_HEADING:
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self._feedback_received = False
                self.state = State.ALIGN_WP8_HEADING  # zurück zur Prüfung

        elif self.state == State.DRIVING_WP9_EAST:
            # P8->P9: Ostfahrt, sanfte Südwand-Korrektur (max 4°), Rückwandstopp
            rear_stop = self.nav_cfg.get('wp9_rear_stop_dist', 0.83)
            seg = self.nav_cfg.get('wp9_segment_length', 0.08)
            self._wall_target = self.nav_cfg.get('wp9_south_wall_target_dist', 0.27)
            rd = self._rear_wall_dist()
            sw, _ = self._south_wall_min()
            if rd is not None:
                self.get_logger().info(
                    f'WP9-Ost: Rückwand={rd:.3f}m  Ziel={rear_stop:.3f}m'
                    + (f'  Südwand={sw:.3f}m' if sw is not None else ''))
            if rd is not None and rd >= rear_stop:
                self.get_logger().info(f'WP9: Rückwandstopp bei {rd:.3f}m, P9 erreicht')
                self._send_cmd('S:0')
                self._feedback_received = False
                self.state = State.NEXT
            elif self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('D:'):
                self._feedback_received = False
                turn_sent = self._wall_angle_correction_strong(negate=True, use_south_wall=True, max_deg=6.0)
                if turn_sent:
                    self.state = State.WAIT_TURN_WP9
                else:
                    self._imu_heading_ref = self._imu_theta_raw
                    self._send_cmd(f'R:{seg:.4f}')

        elif self.state == State.WAIT_TURN_WP9:
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                seg = self.nav_cfg.get('wp9_segment_length', 0.12)
                self._send_cmd(f'R:{seg:.4f}')
                self.state = State.DRIVING_WP9_EAST

        elif self.state == State.WAIT_TURN_WP9_180:
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                self._wall_target = self.nav_cfg.get('wp9_south_wall_target_dist', 0.27)
                self.get_logger().info('WP9: 180°-Drehung fertig, prüfe Südwand-Ausrichtung')
                turn_sent = self._wall_angle_correction_strong(use_south_wall=True, negate=True, max_deg=10.0)
                if turn_sent:
                    self.state = State.WAIT_TURN_WP9
                else:
                    self.get_logger().info('WP9: Ausrichtung OK, starte Ostfahrt')
                    seg = self.nav_cfg.get('wp9_segment_length', 0.12)
                    self._send_cmd(f'R:{seg:.4f}')
                    self.state = State.DRIVING_WP9_EAST

        elif self.state == State.WAIT_TURN_WP10:
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self.get_logger().info('WP10: 90°-Drehung fertig, starte Nordfahrt (+y)')
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                self._send_cmd('R:0.0800')
                self.state = State.DRIVING_WP10_NORTH

        elif self.state == State.DRIVING_WP10_NORTH:
            # P9->P10: +y Fahrt, Frontstopp auf obere_wand (y=0.27)
            front_stop = self.nav_cfg.get('front_stop_wp10', 0.63)
            fd = self._front_wall_dist(half_angle_deg=15)
            if fd is not None:
                self.get_logger().info(f'WP10-Nord: Front={fd:.3f}m  Ziel={front_stop:.3f}m')
            if fd is not None and fd <= front_stop:
                self.get_logger().info(f'WP10: Frontstopp bei {fd:.3f}m, P10 erreicht')
                self._send_cmd('S:0')
                self._feedback_received = False
                self.state = State.NEXT
            elif self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('D:'):
                self._feedback_received = False
                north_ref = self._imu_east_ref + math.pi / 2
                heading_err = 0.0
                if self._imu_theta_raw is not None:
                    heading_err = math.degrees(self._angle_diff(self._imu_theta_raw, north_ref))
                if abs(heading_err) > 2.0:
                    heading_err = max(-10.0, min(10.0, heading_err))
                    corr = -heading_err / 1.03  # negativ: Gegenkorrektur; /1.03: Teensy-Faktor
                    self.get_logger().info(f'WP10-Nord: IMU-Korrektur {corr:.1f}° (Heading-Err={heading_err:.1f}°)')
                    self._send_cmd(f'T:{corr:.1f}')
                    self.state = State.WAIT_TURN_WP10
                else:
                    self._imu_heading_ref = self._imu_theta_raw
                    self._send_cmd('R:0.0800')

        elif self.state == State.WAIT_TURN_WP11:
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('T:'):
                self.get_logger().info('WP11: Drehung fertig, weiter Westfahrt (-x)')
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                self._send_cmd('R:0.1200')
                self.state = State.DRIVING_WP11_EAST

        elif self.state == State.DRIVING_WP11_EAST:
            # P10->P11: -x Fahrt (physisch Ost), untere_wand als linke Referenzwand wie P7->P8
            # _left_wall_min() liest bei -x Fahrt die physisch linke Seite = -y = untere_wand (0.92m)
            # negate=True würde IMU-Heading-Korrektur invertieren → deshalb kein negate
            front_stop = self.nav_cfg.get('front_stop_wp11', 0.215)
            self._wall_target = self.nav_cfg.get('wp11_south_wall_target_dist', 0.92)
            fd = self._front_wall_dist(half_angle_deg=15)
            if fd is not None:
                self.get_logger().info(f'WP11-West: Front={fd:.3f}m  Ziel={front_stop:.3f}m')
            if fd is not None and fd <= front_stop:
                self.get_logger().info(f'WP11: Frontstopp bei {fd:.3f}m, P11 erreicht')
                self._send_cmd('S:0')
                self._feedback_received = False
                self.state = State.NEXT
            elif self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('D:'):
                self._feedback_received = False
                turn_sent = self._wall_angle_correction_strong(max_deg=6.0)
                if turn_sent:
                    self.state = State.WAIT_TURN_WP11
                else:
                    self._imu_heading_ref = self._imu_theta_raw
                    self._send_cmd('R:0.1200')

        elif self.state == State.DRIVING_WP3_SOUTH:
            # Phase 1: nach Süden fahren, vorne auf untere_wand warten
            front_stop = self.nav_cfg.get('wp3_south_stop_dist', 0.35)
            if self._wp3_driving:
                fd = self._front_wall_dist()
                if fd is not None:
                    self.get_logger().info(f'WP3-Süd: Front={fd:.3f}m  Ziel={front_stop:.3f}m')
                if fd is not None and fd <= front_stop:
                    self.get_logger().info(f'WP3-Süd: Stopp bei {fd:.3f}m, drehe 90° nach Westen')
                    self._send_cmd('S:0')
                    time.sleep(0.3)
                    front_angle = self._front_wall_angle()
                    turn_deg = -(90.0 - front_angle)
                    self.get_logger().info(f'WP3-Süd: Drehe {turn_deg:.1f}° (Vorderwand bei {front_angle:.1f}°)')
                    self._feedback_received = False
                    self._send_cmd(f'T:{turn_deg:.1f}')
                    self._segment_driven = 0.0
                    self._pending_drive_dist = self.nav_cfg['segment_length']
                    self.state = State.WAIT_TURN_WP3_WEST
                    return
                if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('D:'):
                    self._feedback_received = False
                    # IMU-Heading-Korrektur: Abweichung von Südrichtung korrigieren
                    if self._imu_heading_ref is not None and self._imu_theta_raw is not None:
                        heading_err = math.degrees(self._angle_diff(self._imu_theta_raw, self._imu_heading_ref))
                        turn_deg = max(-5.0, min(5.0, -heading_err))
                        if abs(turn_deg) > 1.0:
                            self.get_logger().info(f'WP3-Süd: Heading-Korrektur {turn_deg:.1f}° (Drift {heading_err:.1f}°)')
                            self._send_cmd(f'T:{turn_deg:.1f}')
                            self._pending_drive_dist = self.nav_cfg['segment_length']
                            self.state = State.WAIT_TURN_WP3
                            return
                    self._imu_heading_ref = self._imu_theta_raw
                    self._send_cmd(f'R:{self.nav_cfg["segment_length"]:.4f}')
            else:
                # Nach Süden drehen, Winkel anhand Vorderwand korrigieren, +3° Offset
                time.sleep(0.5)  # Scan stabilisieren
                front_angle = self._front_wall_angle()
                turn_deg = -(93.0 - front_angle)
                self.get_logger().info(f'WP3-Süd: Drehe {turn_deg:.1f}° nach Süden (Vorderwand={front_angle:.1f}°, +3° Offset)')
                self._feedback_received = False
                self._send_cmd(f'T:{turn_deg:.1f}')
                # Warte auf T:-Feedback im nächsten Tick (WAIT_TURN_WP3)
                self._wp3_driving = True  # markiert: warte auf T: bevor R: gesendet wird
                self._pending_drive_dist = self.nav_cfg['segment_length']
                self.state = State.WAIT_TURN_WP3

        elif self.state == State.DRIVING_WP3_WEST:
            # Phase 2: nach Westen, untere_wand jetzt links (75°-105°), Wandkorrektur aktiv
            front_stop = self.nav_cfg.get('wp3_west_stop_dist', 0.40)
            fd = self._front_wall_dist(half_angle_deg=10)
            if fd is not None:
                self.get_logger().info(f'WP3-West: Front={fd:.3f}m  Ziel={front_stop:.3f}m')
            if fd is not None and fd <= front_stop:
                self.get_logger().info(f'WP3-West: P3 erreicht bei {fd:.3f}m')
                self._send_cmd('S:0')
                self._wall_active = False
                time.sleep(0.3)
                self.state = State.NEXT
                return
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('D:'):
                self._wall_active = False
                time.sleep(0.15)
                actual = self._odom_dist
                self._odom_dist = 0.0
                self._mcl_update(actual, 0.0)
                self._wall_angle_correction_wp3_west()
                self._odom_dist = 0.0
                self._wall_active = True
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                self._send_cmd(f'R:{self.nav_cfg["segment_length"]:.4f}')

        elif self.state == State.DRIVING_WP4_WEST:
            # P3->P4: gleiche Richtung West, untere_wand links (75°-105°), Wandkorrektur aktiv
            front_stop = self.nav_cfg.get('wp4_west_stop_dist', 0.25)
            if not self._wp3_driving:
                # Erster Eintritt: erstes Segment starten
                self._wp3_driving = True
                self._wall_active = True
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                self._send_cmd(f'R:{self.nav_cfg["segment_length"]:.4f}')
                return
            fd = self._front_wall_dist(half_angle_deg=10)
            if fd is not None:
                self.get_logger().info(f'WP4-West: Front={fd:.3f}m  Ziel={front_stop:.3f}m')
            if fd is not None and fd <= front_stop:
                self.get_logger().info(f'WP4-West: P4 erreicht bei {fd:.3f}m, drehe 90° rechts')
                self._send_cmd('S:0')
                self._wall_active = False
                time.sleep(0.3)
                self._feedback_received = False
                self._send_cmd('T:-90.0')
                self.state = State.WAIT_TURN_WP4
                return
            if self._feedback_received and self._feedback_value is not None and self._feedback_value.startswith('D:'):
                self._wall_active = False
                time.sleep(0.15)
                actual = self._odom_dist
                self._odom_dist = 0.0
                self._mcl_update(actual, 0.0)
                self._wall_angle_correction_wp3_west()
                self._odom_dist = 0.0
                self._wall_active = True
                self._imu_heading_ref = self._imu_theta_raw
                self._feedback_received = False
                self._send_cmd(f'R:{self.nav_cfg["segment_length"]:.4f}')

        elif self.state == State.DRIVING_WP6_DIRECT:
            # P5->P6: 90° links drehen (Nordwand-korrigiert), dann R: mit Lidar-Frontstopp
            if not self._wp3_driving:
                # Erster Eintritt: Nordwand liegt vorne (±45°) → Winkel kompensieren
                self._wp3_driving = True
                self._feedback_received = False
                if self._imu_heading_ref is not None and self._imu_theta_raw is not None:
                    heading_err = math.degrees(self._angle_diff(self._imu_theta_raw, self._imu_heading_ref))
                else:
                    heading_err = 0.0
                turn_deg = (90.0 - heading_err) / 1.03
                self.get_logger().info(f'WP6: Drehe {turn_deg:.1f}° nach links (IMU-Drift {heading_err:.1f}°, 1.03-Vorkorrektur → actual ≈90°)')
                self._send_cmd(f'T:{turn_deg:.1f}')
                self.state = State.WAIT_TURN_WP6
                return
            # Lidar-Frontstopp nur bei Segment-Ende (D:) — NICHT bei jedem Timer-Tick.
            # Kontinuierliche Prüfung triggert sofort nach Drehung auf Scan-Artefakte
            # (Lidar liest ~1.42m direkt nach 90°-Turn obwohl Wand 1.98m weg ist).
            # Gleiche Logik wie generischer DRIVING-State mit front_stop_min_driven.
            front_stop = self.nav_cfg.get('front_stop_wp6', 1.20)
            seg = self.nav_cfg.get('wp6_segment_length', 0.12)
            if self._feedback_received and self._feedback_value is not None and (
                    self._feedback_value.startswith('D:') or self._feedback_value.startswith('T:')):
                self._feedback_received = False
                fd = self._front_wall_dist(half_angle_deg=2)
                if fd is not None:
                    self.get_logger().info(f'WP6-Fahrt: Front={fd:.3f}m  Ziel={front_stop:.3f}m')
                if fd is not None and fd <= front_stop:
                    self.get_logger().info(f'WP6: Frontstopp bei {fd:.3f}m')
                    self._send_cmd('S:0')
                    self._feedback_received = False
                    self.state = State.NEXT
                    return
                self._send_cmd(f'R:{seg:.4f}')

        elif self.state == State.STOP_UPDATE:
            self._stop()
            time.sleep(0.3)
            # Finales MCL-Update im Stillstand (bessere Messung)
            if self._scan_ranges is not None:
                ox = self.robot_cfg['lidar_offset_x']
                oy = self.robot_cfg['lidar_offset_y']
                self.mcl.measurement_update(
                    self._scan_ranges, self._scan_angles, ox, oy)
                self.mcl.resample()
                x, y, theta = self.mcl.estimate_pose()
                self.pose = (x, y, theta)
                self._imu_theta = theta

            self.state = State.CHECK_POSE

        elif self.state == State.CHECK_POSE:
            wp = self.waypoints[self._wp_index]
            err = math.hypot(wp['x'] - self.pose[0], wp['y'] - self.pose[1])
            tol_full = self.nav_cfg['goal_tolerance_full']
            tol_half = self.nav_cfg['goal_tolerance_half']

            if err <= tol_full:
                self.get_logger().info(
                    f'P{wp["id"]}: VOLL ({err*100:.1f}cm) ✓')
                self.state = State.NEXT
            elif err <= tol_half:
                self.get_logger().info(
                    f'P{wp["id"]}: HALB ({err*100:.1f}cm) — versuche Korrektur')
                self._correction_attempts = 0
                self.state = State.CORRECT
            else:
                if self._correction_attempts < self.nav_cfg['max_correction_attempts']:
                    self.get_logger().warn(
                        f'P{wp["id"]}: VERFEHLT ({err*100:.1f}cm) — Korrektur {self._correction_attempts+1}')
                    self._correction_attempts += 1
                    self.state = State.CORRECT
                else:
                    self.get_logger().error(
                        f'P{wp["id"]}: aufgegeben nach {self._correction_attempts} Versuchen')
                    self.state = State.NEXT

        elif self.state == State.CORRECT:
            wp = self.waypoints[self._wp_index]
            dx = wp['x'] - self.pose[0]
            dy = wp['y'] - self.pose[1]
            dist = math.hypot(dx, dy)
            target_theta = math.atan2(dy, dx)
            dtheta = self._angle_diff(target_theta, self.pose[2])

            # Erst ausrichten, dann fahren
            if abs(dtheta) > np.deg2rad(5):
                self._send_cmd(f'T:{math.degrees(dtheta):.2f}')
            else:
                self._send_cmd(f'D:{dist:.4f}')
            self._odom_dist = 0.0
            self._odom_theta = 0.0
            self.state = State.WAIT_CORRECT

        elif self.state == State.WAIT_CORRECT:
            if self._feedback_received:
                self._mcl_update(self._odom_dist, self._odom_theta)
                self._odom_dist = 0.0
                self._odom_theta = 0.0
                self.state = State.STOP_UPDATE

        elif self.state == State.NEXT:
            self._correction_attempts = 0
            # Pose auf erreichten Waypoint setzen (MCL unzuverlässig)
            reached_wp = self.waypoints[self._wp_index]
            current_theta = self._imu_theta_raw if self._imu_theta_raw is not None else self.pose[2]
            self.pose = (reached_wp['x'], reached_wp['y'], current_theta)
            self.mcl.init_pose(reached_wp['x'], reached_wp['y'], current_theta)
            self.get_logger().info(
                f'Pose auf P{reached_wp["id"]} gesetzt: ({reached_wp["x"]:.2f}, {reached_wp["y"]:.2f})')

            self._wp_index += 1
            self._segment_driven = 0.0
            max_wp = self.nav_cfg.get('max_waypoint_index', len(self.waypoints) - 1)

            if self._wp_index >= len(self.waypoints) or self._wp_index > max_wp:
                self.get_logger().info('Alle Waypoints erreicht!')
                self._stop()
                self.state = State.DONE
                with open(self._log_path, 'wb') as f:
                    pickle.dump(self._particle_log, f)
            else:
                wp = self.waypoints[self._wp_index]
                self.get_logger().info(
                    f'Naechster Waypoint: P{wp["id"]} ({wp["x"]:.2f}, {wp["y"]:.2f})')
                wp3_segments = self.nav_cfg.get('wp3_nav_segments', [])
                wp4_segments = self.nav_cfg.get('wp4_nav_segments', [])
                wp6_segments = self.nav_cfg.get('wp6_nav_segments', [])
                wp7_segments = self.nav_cfg.get('wp7_nav_segments', [])
                if wp['id'] in wp3_segments:
                    self._wp3_phase = 'south'
                    self._wp3_driving = False
                    self._segment_driven = 0.0
                    self._feedback_received = False
                    self.state = State.DRIVING_WP3_SOUTH
                elif wp['id'] in wp4_segments:
                    self._wp3_driving = False
                    self._segment_driven = 0.0
                    self._feedback_received = False
                    self.state = State.DRIVING_WP4_WEST
                elif wp['id'] in wp6_segments:
                    self._wp3_driving = False
                    self._feedback_received = False
                    self._imu_heading_ref = self._imu_theta_raw
                    self.state = State.DRIVING_WP6_DIRECT
                elif wp['id'] in wp7_segments:
                    self._wp3_driving = False
                    self._feedback_received = False
                    time.sleep(0.3)
                    # 90° links nach Süden, IMU-Drift seit WP6-Drehung kompensieren
                    if self._imu_heading_ref is not None and self._imu_theta_raw is not None:
                        heading_err = math.degrees(self._angle_diff(self._imu_theta_raw, self._imu_heading_ref))
                    else:
                        heading_err = 0.0
                    turn_deg = (90.0 - heading_err) / 1.03
                    self.get_logger().info(f'WP7-Süd: Drehe {turn_deg:.1f}° nach Süden (IMU-Drift {heading_err:.1f}°, 1.03-Vorkorrektur → actual ≈90°)')
                    self._send_cmd(f'T:{turn_deg:.1f}')
                    self.state = State.WAIT_TURN_WP7_SOUTH
                elif wp['id'] in self.nav_cfg.get('wp8_nav_segments', []):
                    self._feedback_received = False
                    self._imu_heading_ref = self._imu_theta_raw
                    self._imu_glitch_detected = False  # Glitch-Flag für neue WP8-Fahrt zurücksetzen
                    self._imu_glitch_ref = None
                    self._wall_target = self.nav_cfg.get('wp8_south_wall_target_dist', 0.27)
                    self.get_logger().info('WP8: Prüfe Südwand-Ausrichtung vor Westfahrt')
                    turn_sent = self._wall_angle_correction_strong(max_deg=10.0)
                    if turn_sent:
                        self.state = State.WAIT_TURN_WP8
                    else:
                        self.get_logger().info('WP8: Ausrichtung OK, starte Westfahrt')
                        seg = self.nav_cfg.get('wp8_segment_length', 0.12)
                        self._send_cmd(f'R:{seg:.4f}')
                        self.state = State.DRIVING_WP8_WEST
                elif wp['id'] in self.nav_cfg.get('wp9_nav_segments', []):
                    self._feedback_received = False
                    time.sleep(0.3)
                    # Bei BNO055-Glitch während WP8: letzten guten IMU-Wert verwenden
                    if self._imu_glitch_detected and self._imu_glitch_ref is not None:
                        imu_to_use = self._imu_glitch_ref
                        self.get_logger().warn(
                            f'WP9: BNO055-Glitch → nutze Pre-Glitch IMU {math.degrees(imu_to_use):.1f}° (aktuell {math.degrees(self._imu_theta_raw):.1f}°)')
                    else:
                        imu_to_use = self._imu_theta_raw
                    if imu_to_use is not None:
                        turn_deg = math.degrees(self._angle_diff(self._imu_east_ref, imu_to_use))
                        if turn_deg > 90.0:
                            turn_deg -= 360.0
                        turn_deg = max(-210.0, min(-150.0, turn_deg))
                    else:
                        turn_deg = -180.0
                    turn_sent = turn_deg / 1.03  # Teensy-Faktor 1.03 vorausgleichen
                    self.get_logger().info(
                        f'WP9: IMU={math.degrees(self._imu_theta_raw):.1f}°  Ost-Ref={math.degrees(self._imu_east_ref):.1f}°  Turn={turn_deg:.1f}°  Gesendet={turn_sent:.1f}°')
                    self._send_cmd(f'T:{turn_sent:.1f}')
                    self.state = State.WAIT_TURN_WP9_180
                elif wp['id'] in self.nav_cfg.get('wp10_nav_segments', []):
                    self._feedback_received = False
                    time.sleep(0.3)
                    heading_err = 0.0
                    if self._imu_theta_raw is not None:
                        heading_err = math.degrees(self._angle_diff(self._imu_theta_raw, self._imu_east_ref))
                        heading_err = max(-20.0, min(20.0, heading_err))
                    adjusted_turn = 90.0 - heading_err
                    turn_sent = adjusted_turn / 1.03  # Teensy-Faktor vorausgleichen
                    self.get_logger().info(
                        f'WP10: IMU={math.degrees(self._imu_theta_raw):.1f}°  Ost-Err={heading_err:.1f}°  Turn={adjusted_turn:.1f}°  Gesendet={turn_sent:.1f}°')
                    self._send_cmd(f'T:{turn_sent:.1f}')
                    self.state = State.WAIT_TURN_WP10
                elif wp['id'] in self.nav_cfg.get('wp11_nav_segments', []):
                    self._feedback_received = False
                    time.sleep(0.3)
                    north_ref = self._imu_east_ref + math.pi / 2
                    heading_err = 0.0
                    if self._imu_theta_raw is not None:
                        heading_err = math.degrees(self._angle_diff(self._imu_theta_raw, north_ref))
                        heading_err = max(-20.0, min(20.0, heading_err))
                    adjusted_turn = 90.0 - heading_err
                    turn_sent = adjusted_turn / 1.03  # Teensy-Faktor vorausgleichen
                    self.get_logger().info(
                        f'WP11: IMU={math.degrees(self._imu_theta_raw):.1f}°  Nord-Err={heading_err:.1f}°  Turn={adjusted_turn:.1f}°  Gesendet={turn_sent:.1f}°')
                    self._send_cmd(f'T:{turn_sent:.1f}')
                    self.state = State.WAIT_TURN_WP11
                else:
                    self.state = State.ALIGN

        elif self.state == State.DONE:
            pass  # Fertig

    def destroy_node(self):
        self._stop()
        self._csv.close()
        with open(self._log_path, 'wb') as f:
            pickle.dump(self._particle_log, f)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
