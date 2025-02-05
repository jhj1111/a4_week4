import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.target = 'rccar'
        self.subscription = self.create_subscription(
            Image,
            'processed_image',
            self.listener_callback,
            10)
        self.class_id_subscriber = self.create_subscription(String, 'class_id', self.dectector, 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Processed Image", cv_image)
        cv2.waitKey(1)

    def dectector(self, msg):
        self.get_logger().info('dectected')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        rclpy.spin(node)  # Blocking spin, no custom stop mechanism
    finally:
        node.destroy_node()
        rclpy.shutdown()
