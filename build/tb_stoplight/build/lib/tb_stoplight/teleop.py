import rclpy
from rclpy.node import Node
import tty
import sys
import termios
from termios import TCIOFLUSH, TCSADRAIN
import select
from geometry_msgs.msg import Twist

WAFFLE_MAX_LINEAR_VELOCITY = 0.26
WAFFLE_MAX_ANGULAR_VELOCITY = 1.82
LINEAR_VELOCITY_STEP_SIZE = 0.01
ANGULAR_VELOCITY_STEP_SIZE = 0.1

class TeleopControl(Node):
    def __init__(self):
        super().__init__('teleop_control')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.read_input()

    def read_input(self):
        self.get_logger().info(f'Now accepting data...')
        current_linear_velocity = 0.0
        current_angular_velocity = 0.0
        twist = Twist()
        while(True):
            key = read_key()
            if key == 'w':
                if current_linear_velocity + LINEAR_VELOCITY_STEP_SIZE < WAFFLE_MAX_LINEAR_VELOCITY:
                    current_linear_velocity += LINEAR_VELOCITY_STEP_SIZE
                else:
                    current_linear_velocity = WAFFLE_MAX_LINEAR_VELOCITY
            elif key == 's':
                if current_linear_velocity - LINEAR_VELOCITY_STEP_SIZE > -WAFFLE_MAX_LINEAR_VELOCITY:
                    current_linear_velocity -= LINEAR_VELOCITY_STEP_SIZE
                else:
                    current_linear_velocity = -WAFFLE_MAX_LINEAR_VELOCITY
            elif key == 'a':
                if current_angular_velocity + ANGULAR_VELOCITY_STEP_SIZE < WAFFLE_MAX_ANGULAR_VELOCITY:
                    current_angular_velocity += ANGULAR_VELOCITY_STEP_SIZE
                else:
                    current_angular_velocity = WAFFLE_MAX_ANGULAR_VELOCITY
            elif key == 'd':
                if current_angular_velocity - ANGULAR_VELOCITY_STEP_SIZE > -WAFFLE_MAX_ANGULAR_VELOCITY:
                    current_angular_velocity -= ANGULAR_VELOCITY_STEP_SIZE
                else:
                    current_angular_velocity = -WAFFLE_MAX_ANGULAR_VELOCITY
            elif key == ' ':
                current_angular_velocity = 0.0
                current_linear_velocity = 0.0
            elif key == 'p':
                # Literal escape (esc) character
                break
        
            current_linear_velocity = round(current_linear_velocity, 2)
            current_angular_velocity = round(current_angular_velocity, 2)

            twist.linear.x = current_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = current_angular_velocity

            self.publisher.publish(twist)
            self.get_logger().info(f'LINEAR: {current_linear_velocity}, ANGULAR: {current_angular_velocity}')

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

def main():
    rclpy.init()
    teleop_control = TeleopControl()
    rclpy.spin(teleop_control)
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()