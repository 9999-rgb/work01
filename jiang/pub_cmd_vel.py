#!/usr/bin/env python3
"""持续发布 cmd_vel 数据，供 zenoh bridge 转发验证"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.2, self.publish)  # 5Hz
        self.angle = 0.0
        self.get_logger().info('持续发布 /turtle1/cmd_vel (5Hz) ...')

    def publish(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = math.sin(self.angle)
        self.angle += 0.3
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
