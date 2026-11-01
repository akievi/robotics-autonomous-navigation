import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class MeasureNode(Node):
    def __init__(self):
        super().__init__('measure_lane_width')
        self.subscription = self.create_subscription(
            Image, '/camera_image', self.cb, 10)
        self.bridge = CvBridge()

    def cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError:
            return
        height, width = frame.shape[:2]
        mid = width // 2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        roi = mask[int(height * 0.40):, :]
        histogram = np.sum(roi, axis=0)

        # Alle Peaks finden
        threshold = max(500, np.max(histogram) * 0.4)
        above = (histogram >= threshold).astype(np.uint8)
        num_labels, labels = cv2.connectedComponents(above.reshape(1, -1))
        peaks = []
        for label in range(1, num_labels):
            cols = np.where(labels[0] == label)[0]
            peaks.append(int(np.mean(cols)))

        peaks.sort()
        print(f"Peaks bei x={peaks}  |  Abstände: {[peaks[i+1]-peaks[i] for i in range(len(peaks)-1)]}")

        # Visualisierung
        out = frame.copy()
        for p in peaks:
            cv2.line(out, (p, 0), (p, height), (0, 255, 0), 1)
        cv2.imshow('Peaks', out)
        cv2.imshow('Mask', mask)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = MeasureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
