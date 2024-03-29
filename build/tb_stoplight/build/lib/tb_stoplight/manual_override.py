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

def read_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        character = sys.stdin.read(1)
    finally:
        termios.tcflush(fd, termios.TCIOFLUSH)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return character

class ManualStoplightInterpretor(Node):
    def __init__(self):
        super().__init__('manual_stoplight_interpretor')
        self.publisher = self.create_publisher(String, 'override', 10)
        self.manual_override()

    def manual_override(self):
        while(True):
            msg = String()
            key = read_key()
            if key == 'r':
                print('Red')
                msg.data = 'red'
                self.publisher.publish(msg)
            elif key == 'g':
                print('Green')
                msg.data = 'green'
                self.publisher.publish(msg)
            elif key == 'y':
                print('Yellow')
                msg.data = 'yellow'
                self.publisher.publish(msg)
            elif key == 'q':
                break
        
def main():
    rclpy.init()
    interpretor = ManualStoplightInterpretor()
    rclpy.spin(interpretor)
    interpretor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


