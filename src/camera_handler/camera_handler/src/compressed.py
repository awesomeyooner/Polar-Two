import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2 

class CameraCompressedDriver(Node):

    def __init__(self):
        super().__init__('camera_compressor')

        self.publisher = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.listener_callback, 10)

        self.bridge = CvBridge()

    def listener_callback(self, image):
        frame = self.bridge.imgmsg_to_cv2(image)

        light_frame = cv2.resize(
            frame, 
            (
                int(frame.shape[1] * 0.25), 
                int(frame.shape[0] * 0.25)
            )
        )
        compressed = self.bridge.cv2_to_compressed_imgmsg(light_frame, 'jpg')

        self.publisher.publish(compressed)



def main(args=None):
    rclpy.init(args=args)

    camera_driver = CameraCompressedDriver()

    rclpy.spin(camera_driver)

    camera_driver.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()