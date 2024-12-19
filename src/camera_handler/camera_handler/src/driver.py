import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2 

class CameraDriver(Node):

    def __init__(self):
        super().__init__('camera_driver')

        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)

        timer_period = 0.005 #seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.capture = cv2.VideoCapture(0)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.capture.read()

        mirrored_side = cv2.flip(frame, 0)
        normal = cv2.flip(mirrored_side, 1)

        if ret:
            self.publisher.publish(
                self.bridge.cv2_to_imgmsg(normal)
            )

def main(args=None):
    rclpy.init(args=args)
    camera_driver = CameraDriver()

    rclpy.spin(camera_driver)

    camera_driver.destroy_node()

    camera_driver.capture.release()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()