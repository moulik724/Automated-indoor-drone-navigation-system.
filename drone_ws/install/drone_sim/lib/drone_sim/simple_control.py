#!/usr/bin/env python3
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.listen_keyboard)
        self.settings = termios.tcgetattr(sys.stdin)

    def listen_keyboard(self):
        key = self.get_key()
        twist = Twist()

        if key == 'w':
            twist.linear.x = 1.0
        elif key == 's':
            twist.linear.x = -1.0
        elif key == 'a':
            twist.angular.z = 1.0
        elif key == 'd':
            twist.angular.z = -1.0
        elif key == ' ':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher_.publish(twist)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

