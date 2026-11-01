import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class RectangleMarkerNode(Node):
    def __init__(self):
        super().__init__('rectangle_marker_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera_image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        # --- Weißes Rechteck auf schwarzem Hintergrund detektieren ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        # Konturen finden
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        output = frame.copy()

        if contours:
            # Größte Kontur = das weiße Rechteck
            largest = max(contours, key=cv2.contourArea)

            # Mindestgröße prüfen (filtert Rauschen heraus)
            if cv2.contourArea(largest) > 2000:
                # Minimales umschließendes Rechteck (rotiert) berechnen
                rect = cv2.minAreaRect(largest)
                box = cv2.boxPoints(rect)
                box = np.int32(box)

                # Rechteck einzeichnen
                cv2.drawContours(output, [box], 0, (0, 255, 0), 2)

                # Mittelpunkt des Rechtecks
                cx, cy = int(rect[0][0]), int(rect[0][1])
                cv2.circle(output, (cx, cy), 6, (0, 0, 255), -1)

                # Winkel des Rechtecks
                angle = rect[2]
                # minAreaRect gibt Winkel in [-90, 0) zurück — auf [0, 180) normalisieren
                width_r, height_r = rect[1]
                if width_r < height_r:
                    angle = angle + 90

                # Linie entlang der langen Achse des Rechtecks zeichnen
                length = int(max(width_r, height_r) / 2)
                angle_rad = np.deg2rad(angle)
                dx = int(np.cos(angle_rad) * length)
                dy = int(np.sin(angle_rad) * length)

                pt1 = (cx - dx, cy - dy)
                pt2 = (cx + dx, cy + dy)
                cv2.line(output, pt1, pt2, (0, 255, 255), 3)  # gelbe Achslinie

                # Winkel anzeigen
                cv2.putText(output, f'Angle: {angle:.1f} deg',
                            (cx + 12, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow('Rectangle Marker', output)
        cv2.imshow('Mask', mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RectangleMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
