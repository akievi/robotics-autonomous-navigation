import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import glob

def find_camera_index():
    for path in sorted(glob.glob('/sys/class/video4linux/video*/name')):
        with open(path) as f:
            if 'Webcam' in f.read() or 'webcam' in f.read() or 'Camera' in f.read() or 'camera' in f.read() or 'USB' in f.read():
                index = int(path.split('video')[2].split('/')[0])
                cap = cv2.VideoCapture(index)
                if cap.isOpened():
                    cap.release()
                    return index
    return 0

class CameraPublisher(Node):
    def __init__(self):
      super().__init__('camera_publisher')
      self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
      timer_period = 0.016
      self.timer = self.create_timer(timer_period, self.timer_callback)
      self.br = CvBridge()
      index = find_camera_index()
      self.get_logger().info(f'Opening camera at /dev/video{index}')
      self.cap = cv2.VideoCapture(index)
      self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
      self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
      if not self.cap.isOpened():
         self.get_logger().error('Could not open videostream')
         exit(1)
         
    def timer_callback(self):
      ret, frame = self.cap.read()
      if ret:
         msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
         self.publisher_.publish(msg)
      else:
         self.get_logger(),error('Error in capturing image')
            
    def destroy_node(self):
      super().destroy_node()
      self.cap.release()
        
def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
                
