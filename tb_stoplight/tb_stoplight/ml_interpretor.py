import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import tty
import sys
import termios
from termios import TCIOFLUSH, TCSADRAIN
from std_msgs.msg import String
import numpy as np
# from PIL import Image
import os
import tensorflow as tf
from tensorflow.keras import datasets, layers, models
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split

class StoplightInterpretor(Node):
    def __init__(self):
        super().__init__('stoplight_interpretor')
        self.subscriber = self.create_subscription(
            Twist,
            'nav_cmd_vel',
            self.nav_cmd,
            10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            Image,
            'video_frames',
            self.receive_image,
            10
        )
        self.bridge = CvBridge()
        model_path = "/home/william/ETLS699_tb_stoplight/tb_stoplight/CNN_1"
        model = tf.keras.models.load_model(model_path)
        self.prediction_model = models.Sequential([model, layers.Softmax()])

    def receive_image(self, data):
        self.get_logger().info("Got frame")
        frame = self.bridge.imgmsg_to_cv2(data)
        new_size = (180, 180)
        resized_img = cv2.resize(frame, new_size, interpolation=cv2.INTER_AREA)
        images = []
        images.append(resized_img)
        prediction = self.prediction_model.predict(np.array(images))
        print(np.argmax(prediction))

    def nav_cmd(self, data):
        self.get_logger().info(f'Intercepted nav command, PosX:{data.linear.x} PosY:{data.linear.y} PosZ:{data.linear.z}, AngX:{data.angular.x} AngY:{data.angular.y} AngZ:{data.angular.z}')
        twist = Twist()
        twist.linear.x = data.linear.x * self.light
        twist.linear.y = data.linear.y * self.light
        twist.linear.z = data.linear.z * self.light

        twist.angular.x = data.angular.x * self.light
        twist.angular.y = data.angular.y * self.light
        twist.angular.z = data.angular.z * self.light
        self.publisher.publish(twist)

def main():
    rclpy.init()
    interpretor = StoplightInterpretor()
    rclpy.spin(interpretor)
    interpretor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()















