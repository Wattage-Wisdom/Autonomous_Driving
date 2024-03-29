import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.receive_image,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('Initialized')
        self.image_number = 0
    
    def receive_image(self, data):
        self.get_logger().info(f'Received video frame #{self.image_number}')
        frame = self.bridge.imgmsg_to_cv2(data)
        cv2.imshow("camera", frame)
        if self.image_number % 5 == 0 or self.image_number == 0:
            self.get_logger().info(f'Wrote image #{self.image_number // 5}')
            cv2.imwrite(f"./saved_images/pic-{self.image_number // 5}.png", frame)
        self.image_number += 1
        cv2.waitKey(1)

def main():
    rclpy.init()
    receiver = ImageSubscriber()
    rclpy.spin(receiver)
    receiver.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()