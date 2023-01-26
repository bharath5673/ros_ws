#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import datetime
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__('draw_circle')
        self.cmd_vel_pub_ = self.create_publisher(Twist , 'turtle1/cmd_vel', 10)
        self.get_logger().info('Draw circle node has been started')
        self.timer = self.create_timer(0.5, self.send_velocity)

    def send_velocity(self):
        msg = Twist()
        msg.linear.x=2.0
        msg.angular.z=1.0
        self.cmd_vel_pub_.publish(msg)

def main(args=None): 
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()