import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, 'video_frames', 10)
        self.timer = self.create_timer(0.1, self.publish_image) # 0.1 -> 100 ms per tick
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        self.bridge = CvBridge()

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret == True:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(frame))
        self.get_logger().info('Publishing video frame')

def main():
    rclpy.init()
    publisher = ImagePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()
    