import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import math

# Schwellwert für Weißerkennung
THRESHOLD = 190

# ROI: untere 40% des Bildes
ROI_START = 0.2

# Minimale Konturfläche damit eine Kontur als Linie gilt
MIN_CONTOUR_AREA = 100

# Mindestabstand zwischen zwei Linien damit sie als Spurpaar gelten
MIN_LANE_WIDTH_PX = 300

# Maximaler Sprung (px) den eine Linie zwischen zwei Frames machen darf
# um noch als dieselbe Seite erkannt zu werden
MAX_JUMP_PX = 150

# Nach so vielen Frames ohne Treffer auf einer Seite wird deren Memory verworfen
MEMORY_TIMEOUT_FRAMES = 10

# Skalierung: wie stark wird der Winkel auf den Steerwert abgebildet
ANGLE_SCALE = 1

# Maximaler Steerwert der ans Topic weitergegeben wird
MAX_STEERING = 50

# Ab diesem Winkel (zur Senkrechten) gilt eine einzelne sichtbare Linie als
# Signatur einer scharfen 90°/180°-Ecke statt einer normalen Kurve
SHARP_CORNER_ANGLE_DEG = 50

# Steerwert für scharfe Ecken (darf über MAX_STEERING liegen)
SHARP_CORNER_STEERING = 90

# Wie lange die Corner-Signatur stabil anliegen muss bevor eingelenkt wird
CORNER_DELAY_S = 0.2

# Halbe Breite des Lidar-Sichtfensters in Grad (z.B. 10 → Fenster ist 20° breit)
LIDAR_HALF_WIDTH_DEG = 20.0

# Maximale Verschiebung des Lidar-Fensters bei vollem Lenkausschlag (in Grad)
LIDAR_MAX_SHIFT_DEG = 30.0

# Verschiebung bei Sharp-Corner-Steering (horizontale Linie erkannt = ~90° Kurve)
LIDAR_CORNER_SHIFT_DEG = 75.0

# Auf True setzen um ohne Display zu starten (kein cv2.imshow)
HEADLESS = True

OVERTAKE_COOLDOWN = 2.0  # seconds before re-triggering allowed after maneuver


def fit_line(contour, roi_height):
    """
    Gibt (angle_deg, cx) zurück.
    angle_deg: Durchschnittswinkel der gefitteten Linie zur Senkrechten (-90° bis +90°).
    cx: x-Position der Linie auf Höhe des unteren ROI-Rands (nicht Bounding-Box-Mitte!).
        Eine schräge Linie hat eine Bounding-Box-Mitte, die je nach Neigung weit von der
        tatsächlichen Fahrbahnseite abweicht. Die Position am unteren Rand (am nächsten
        zum Roboter) ist die robustere Referenz für "links" oder "rechts".
    """
    if len(contour) < 5:
        return None
    [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
    if abs(vy) < 1e-6:
        angle_deg = 90.0
        cx = float(x0)
    else:
        angle_deg = float(np.degrees(np.arctan2(vx, vy)))
        t = (roi_height - y0) / vy
        cx = float(x0 + t * vx)
    # Normalisieren auf -90° bis +90°
    if angle_deg > 90:
        angle_deg -= 180
    elif angle_deg < -90:
        angle_deg += 180
    return angle_deg, cx


class CorrectionVectorNode(Node):
    def __init__(self):
        super().__init__('correction_vector_node')
        self.subscription = self.create_subscription(
            Image, '/camera_image', self.callback, 10)
        self.publisher = self.create_publisher(Int32, 'micro_ros_fabinal_subscriber', 10)
        self.bridge = CvBridge()
        self.last_left_cx = None
        self.last_right_cx = None
        self.left_miss_count = 0
        self.right_miss_count = 0
        self.corner_pending_since = None
        self.obstacle_detected = False
        self.create_subscription(Int32, 'obstacle_detected', self.obstacle_callback, 10)
        self._lidar_param_client = self.create_client(
            SetParameters, '/lidar_filter/set_parameters')
        self._last_lidar_center = None
        # Raw scan for side detection
        self._last_raw_scan = None
        self.create_subscription(LaserScan, '/scan', self._raw_scan_callback,
                                 rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        # Overtake state
        self._overtaking = False
        self._overtake_trigger_value = 200
        self._overtake_cooldown_until = 0.0
        self.create_subscription(Int32, 'overtake_done', self._overtake_done_callback, 10)

    def _raw_scan_callback(self, msg):
        self._last_raw_scan = msg

    def _detect_open_side(self):
        if self._last_raw_scan is None:
            return 1
        msg = self._last_raw_scan
        # Find the angle of the closest finite reading within ±45°
        closest_r = 999.0
        closest_angle = 0.0
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            if -LIDAR_HALF_WIDTH_DEG <= angle_deg <= LIDAR_HALF_WIDTH_DEG and r < closest_r:
                closest_r = r
                closest_angle = angle_deg
        # Obstacle right of center (positive angle) → overtake left (-1), and vice versa
        return 1 if closest_angle >= 0.0 else -1

    def obstacle_callback(self, msg):
        now = time.monotonic()
        if self._overtaking:
            return
        if msg.data == 1 and now > self._overtake_cooldown_until:
            direction = self._detect_open_side()
            self._overtake_trigger_value = 200 if direction > 0 else -200
            out_msg = Int32()
            out_msg.data = self._overtake_trigger_value
            self.publisher.publish(out_msg)
            self._overtaking = True
            self.get_logger().info(
                f'OVERTAKE TRIGGER — {"RIGHT" if direction > 0 else "LEFT"} ({self._overtake_trigger_value})')

    def _overtake_done_callback(self, msg):
        if msg.data == 1 and self._overtaking:
            self._overtaking = False
            self._overtake_cooldown_until = time.monotonic() + OVERTAKE_COOLDOWN
            self.last_left_cx = None
            self.last_right_cx = None
            self.get_logger().info('OVERTAKE COMPLETE — resuming lane following')

    def _update_lidar_angle(self, steering):
        if not self._lidar_param_client.service_is_ready():
            return
        # Sharp corner: shift to 30-40° window; otherwise proportional shift up to 5°
        if abs(steering) > MAX_STEERING:
            center = -35.0 if steering > 0 else 35.0
        else:
            center = -(steering / MAX_STEERING) * 5.0
        center = round(center, 1)
        if center == self._last_lidar_center:
            return
        self._last_lidar_center = center
        angle_begin = center - LIDAR_HALF_WIDTH_DEG
        angle_end = center + LIDAR_HALF_WIDTH_DEG

        def make_param(name, value):
            p = Parameter()
            p.name = name
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
            return p

        req = SetParameters.Request()
        req.parameters = [make_param('angle_begin', angle_begin), make_param('angle_end', angle_end)]
        self._lidar_param_client.call_async(req)

    def callback(self, msg):
        if self._overtaking:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        height, width = frame.shape[:2]
        mid = width // 2
        roi_y = int(height * ROI_START)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, THRESHOLD, 255, cv2.THRESH_BINARY)
        roi = mask[roi_y:, :]

        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        roi_height = height - roi_y

        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA:
                continue
            result = fit_line(cnt, roi_height)
            if result is None:
                continue
            angle_deg, cx = result
            candidates.append((angle_deg, cx))
        candidates.sort(key=lambda x: x[1])

        if self.last_left_cx is None and self.last_right_cx is None:
            # Kein Memory vorhanden (erster Frame) → Fallback auf Mid-Split
            left_candidates = [c for c in candidates if c[1] < mid]
            right_candidates = [c for c in candidates if c[1] >= mid]
            left = max(left_candidates, key=lambda x: x[1]) if left_candidates else None
            right = min(right_candidates, key=lambda x: x[1]) if right_candidates else None
        else:
            # Jede Seite bekommt nur eine Kontur zugewiesen, wenn diese nah
            # genug an der zuletzt bekannten Position dieser Seite liegt.
            # Verhindert, dass eine von oben einwandernde gegenüberliegende
            # Linie als die fehlende Linie der anderen Seite missinterpretiert wird.
            left = None
            right = None
            if self.last_left_cx is not None:
                left_pool = [c for c in candidates if abs(c[1] - self.last_left_cx) <= MAX_JUMP_PX]
                if left_pool:
                    left = min(left_pool, key=lambda x: abs(x[1] - self.last_left_cx))
            if self.last_right_cx is not None:
                right_pool = [c for c in candidates
                               if abs(c[1] - self.last_right_cx) <= MAX_JUMP_PX and c is not left]
                if right_pool:
                    right = min(right_pool, key=lambda x: abs(x[1] - self.last_right_cx))

            # Falls eine Seite kein Memory hatte (z.B. war vorher nur eine Linie da),
            # mit Mid-Split relativ zur jeweils anderen, bereits zugeordneten Linie auffüllen
            if self.last_left_cx is None and left is None:
                left_pool = [c for c in candidates if c[1] < mid and c is not right]
                left = max(left_pool, key=lambda x: x[1]) if left_pool else None
            if self.last_right_cx is None and right is None:
                right_pool = [c for c in candidates if c[1] >= mid and c is not left]
                right = min(right_pool, key=lambda x: x[1]) if right_pool else None

        # Abstandsprüfung: Paar nur gültig wenn Abstand groß genug
        if left is not None and right is not None:
            if right[1] - left[1] < MIN_LANE_WIDTH_PX:
                # Zu nah beieinander → nur die zur Mitte nächste behalten
                if mid - left[1] < right[1] - mid:
                    right = None
                else:
                    left = None

        # Memory aktualisieren, mit Timeout falls eine Seite länger fehlt
        if left is not None:
            self.last_left_cx = left[1]
            self.left_miss_count = 0
        else:
            self.left_miss_count += 1
            if self.left_miss_count >= MEMORY_TIMEOUT_FRAMES:
                self.last_left_cx = None

        if right is not None:
            self.last_right_cx = right[1]
            self.right_miss_count = 0
        else:
            self.right_miss_count += 1
            if self.right_miss_count >= MEMORY_TIMEOUT_FRAMES:
                self.last_right_cx = None

        # Lenkwert aus Winkeldifferenz berechnen
        # Gerade: |L| ≈ R → error ≈ 0
        # Nach rechts geneigt: |L| < R → error negativ → nach links korrigieren
        # Nach links geneigt: |L| > R → error positiv → nach rechts korrigieren
        now = time.monotonic()

        def crosses_to_center(line):
            angle_deg, cx = line
            if abs(angle_deg) < SHARP_CORNER_ANGLE_DEG:
                return False
            return (cx > mid and angle_deg < 0) or (cx < mid and angle_deg > 0)

        def exits_same_side(line):
            angle_deg, cx = line
            if abs(angle_deg) < SHARP_CORNER_ANGLE_DEG:
                return False
            return (cx > mid and angle_deg > 0) or (cx < mid and angle_deg < 0)

        def trigger_corner(pending_since, fallback_steering):
            if pending_since is None:
                pending_since = now
            if now - pending_since >= CORNER_DELAY_S:
                return pending_since, True
            return pending_since, False

        if left is not None and right is not None:
            error = abs(left[0]) - abs(right[0])
            steering = int(np.clip(-error * ANGLE_SCALE, -MAX_STEERING, MAX_STEERING))
            status = f'BOTH  L:{left[0]:.1f}° R:{right[0]:.1f}° err:{error:.1f}°'
            self.corner_pending_since = None
        elif left is not None:
            if exits_same_side(left) or crosses_to_center(left):
                self.corner_pending_since = None
                steering = SHARP_CORNER_STEERING-30
                status = f'SHARP CORNER (LEFT)  angle:{left[0]:.1f}°'
            elif abs(left[0]) >= SHARP_CORNER_ANGLE_DEG:
                self.corner_pending_since, fired = trigger_corner(self.corner_pending_since, int(MAX_STEERING * 0.5))
                if fired:
                    steering = SHARP_CORNER_STEERING-30
                    status = f'SHARP CORNER (LEFT)  angle:{left[0]:.1f}°'
                else:
                    steering = int(MAX_STEERING * 0.5)
                    status = f'SHARP CORNER PENDING (LEFT)  angle:{left[0]:.1f}°  wait:{CORNER_DELAY_S - (now - self.corner_pending_since):.2f}s'
            else:
                self.corner_pending_since = None
                steering = int(MAX_STEERING * 0.5)
                status = f'LEFT ONLY  angle:{left[0]:.1f}°'
        elif right is not None:
            if exits_same_side(right) or crosses_to_center(right):
                self.corner_pending_since = None
                steering = -SHARP_CORNER_STEERING+20
                status = f'SHARP CORNER (RIGHT)  angle:{right[0]:.1f}°'
            elif abs(right[0]) >= SHARP_CORNER_ANGLE_DEG:
                self.corner_pending_since, fired = trigger_corner(self.corner_pending_since, -MAX_STEERING)
                if fired:
                    steering = -SHARP_CORNER_STEERING+20
                    status = f'SHARP CORNER (RIGHT)  angle:{right[0]:.1f}°'
                else:
                    steering = -MAX_STEERING
                    status = f'SHARP CORNER PENDING (RIGHT)  angle:{right[0]:.1f}°  wait:{CORNER_DELAY_S - (now - self.corner_pending_since):.2f}s'
            else:
                self.corner_pending_since = None
                steering = -int(MAX_STEERING * 0.5)
                status = f'RIGHT ONLY  angle:{right[0]:.1f}°'
        else:
            self.corner_pending_since = None
            horiz_candidates = [c for c in candidates if abs(c[0]) >= 70]
            if horiz_candidates:
                # Horizontale Linie ohne Seitenlinien: drehe weg von der Seite wo die
                # horizontale liegt — die andere Seite ist der freie Weg.
                horiz = max(horiz_candidates, key=lambda c: abs(c[0]))
                if horiz[1] < mid:
                    steering = SHARP_CORNER_STEERING
                    status = f'HORIZ LEFT → drehe rechts  cx:{horiz[1]:.0f}'
                else:
                    steering = -SHARP_CORNER_STEERING
                    status = f'HORIZ RIGHT → drehe links  cx:{horiz[1]:.0f}'
            else:
                steering = 0
                status = 'NONE'

        out_msg = Int32()
        out_msg.data = steering
        self.publisher.publish(out_msg)
        self.get_logger().info(status + f'  steer:{steering}')
        # Update lidar window — skip for obstacle stop value (500)
        if abs(steering) < 500:
            self._update_lidar_angle(steering)

        if not HEADLESS:
            # Visualisierung
            out = frame.copy()
            cv2.line(out, (0, roi_y), (width, roi_y), (50, 50, 50), 1)

            for cnt in contours:
                if cv2.contourArea(cnt) < MIN_CONTOUR_AREA:
                    continue
                cv2.drawContours(out, [cnt + np.array([0, roi_y])], -1, (60, 60, 60), 1)

            for line, color in [(left, (255, 100, 0)), (right, (0, 100, 255))]:
                if line is not None:
                    angle_deg, cx = line
                    half_h = (height - roi_y) // 2
                    x_top = int(cx - np.tan(np.radians(angle_deg)) * half_h)
                    x_bot = int(cx + np.tan(np.radians(angle_deg)) * half_h)
                    cv2.line(out, (x_top, roi_y), (x_bot, height), color, 2)

            # Pfeil: zeigt Richtung der Korrektur
            arrow_target_x = int(np.clip(mid + steering * mid // 100, 0, width - 1))
            cv2.circle(out, (mid, height - 10), 10, (0, 0, 255), -1)
            cv2.arrowedLine(out, (mid, height - 10), (arrow_target_x, roi_y + 20),
                            (0, 255, 255), 3, tipLength=0.3)
            cv2.putText(out, f'[{status}]  steer:{steering}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            cv2.imshow('Correction Vector', out)

            # Debug
            debug = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                m = cv2.moments(cnt)
                if m['m00'] < 1:
                    continue
                cx_d = int(m['m10'] / m['m00'])
                cy_d = int(m['m01'] / m['m00'])
                cv2.drawContours(debug, [cnt], -1, (0, 255, 0), 1)
                cv2.putText(debug, f'{int(area)}', (cx_d, cy_d),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
            cv2.imshow('Debug Contours', debug)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CorrectionVectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if not HEADLESS:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
