#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import datetime

class MyNode(Node):
    def __init__(self):
        super().__init__('first_node')
        self.get_logger().info('Hello from ROS2 - testing nodes..!')

        self.counter_ = 0
        self.create_timer(1.0,self.timer_calback)

    def timer_calback(self):
        self.get_logger().info('hello...'+ str(self.counter_)+ ' \n'+ str(datetime.datetime.now()))
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()