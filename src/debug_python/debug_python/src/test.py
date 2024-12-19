import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
import numpy as np

class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')

        self.publisher = self.create_publisher(String, 'my_topic', 10)

        timer_period = 0.005 #seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        message = String()

        message.data = "hi"
        print(np.__file__)

        self.publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    camera_driver = TestNode()

    rclpy.spin(camera_driver)

    camera_driver.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()