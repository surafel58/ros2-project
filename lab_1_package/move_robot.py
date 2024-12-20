#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Create publisher for cmd_vel topic
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Robot controller initialized')
        self.get_logger().info("""
        Control the robot using keyboard:
        ---------------------------
        Moving around:
        w
        a  s  d
        w/s : increase/decrease linear velocity
        a/d : increase/decrease angular velocity
        space key : force stop
        CTRL-C to quit
        """)
        # Initialize velocities
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def get_key(self):
        # Get keyboard input
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def move_robot(self):
        # Create and publish Twist message
        msg = Twist()
        msg.linear._x = self.linear_vel
        msg.angular._z = self.angular_vel
        self.publisher.publish(msg)

    def update_velocity(self, key):
        step = 0.1
        if key == 'w':
            self.linear_vel += step
        elif key == 's':
            self.linear_vel -= step
        elif key == 'a':
            # Increase left wheel speed only
            self.linear_vel += step/2
            self.angular_vel += step
        elif key == 'd':
            self.angular_vel -= step
        elif key == ' ':
            self.linear_vel = 0.0
            self.angular_vel = 0.0

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
    try:
        while True:
            key = controller.get_key()
            if key == '\x03':  # CTRL-C
                break
            controller.update_velocity(key)
            controller.move_robot()
    except Exception as e:
        print(e)
    finally:
        # Stop the robot
        stop_msg = Twist()
        controller.publisher.publish(stop_msg)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
