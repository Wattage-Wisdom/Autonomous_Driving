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

#Color boundaries for red
Rlower = [0, 0, 145] # lower boundaries
Rupper = [145, 145, 255] # upper boundaries
Rlower = np.array(Rlower, dtype = "uint8")
Rupper = np.array(Rupper, dtype = "uint8")

#Color boundaries for green
Glower = [0, 140, 0] # lower boundaries
Gupper = [140, 255, 140] # upper boundaries
Glower = np.array(Glower, dtype = "uint8")
Gupper = np.array(Gupper, dtype = "uint8")

#Image row and color boundaries based on image shape
imin = 0
imax = 111
jmin = 0
jmax = 159
step = 1
threshold = 1

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

class StoplightInterpretor(Node):
    def __init__(self):
        super().__init__('stoplight_interpretor')
        self.subscriber = self.create_subscription(
            Twist,
            'nav_cmd_vel',
            self.nav_cmd,
            10
        )
        self.subscriber = self.create_subscription(
            String,
            'override',
            self.manual_override,
            10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.subscriber = self.create_subscription(
        #     Image,
        #     'video_frames',
        #     self.receive_image,
        #     10
        # )
        self.bridge = CvBridge()
        self.green_semaphore = 0
        self.red_semaphore = 0
        self.light = 0

    def manual_override(self, msg):
        if msg.data == 'red':
            print('Red')
            self.light = 0
        elif msg.data == 'green':
            print('Green')
            self.light = 1
        elif msg.data == 'yellow':
            print('Yellow')
            self.light = 0.5
        
    def receive_image(self, data):
        # self.get_logger().info('Got image')
        frame = self.bridge.imgmsg_to_cv2(data)
        # Resize image
        frame = cv2.resize(frame, (160,112))

        # Create Masks
        maskR = cv2.inRange(frame, Rlower, Rupper) #Red boundaries mask
        redMask = cv2.bitwise_and(frame, frame, mask = maskR) #Red filtered image
        maskG = cv2.inRange(frame, Glower, Gupper) #Green boundaries mask
        greenMask = cv2.bitwise_and(frame, frame, mask = maskG) #Green filtered image

        # Reset Semaphore variables to zero
        self.green_semaphore = 0
        self.red_semaphore = 0

        #Look for green Semaphores
        for i in range(imin, imax, step):
            for j in range(jmin, jmax, step):
                if greenMask[i][j][1] > 0: #If that pixel is green
                    self.green_semaphore += 1 #Add a greenSemaphore
    
        #Look for red Semaphores
        for i in range(imin, imax, step):
            for j in range(jmin, jmax, step):
                if redMask[i][j][2] > 0: #If that pixel is red
                    self.red_semaphore += 1 #Add a redSemaphore
        
        if (self.red_semaphore > 0 and self.green_semaphore > 0):
            if self.red_semaphore >= self.green_semaphore*1.5:
                self.get_logger().info('Red_set')
            elif self.green_semaphore >= self.red_semaphore*1.5:
                self.get_logger().info('Green_set')
            else:
                self.get_logger().info('Yellow')
        else:
            self.get_logger().info('NONE')

        print('Green Semaphores detected: ', self.green_semaphore)
        print('Red Semaphores detected: ', self.red_semaphore)

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


