#!/usr/bin/python3
import os, sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, String, Int32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
bridge = CvBridge()

import threading

vel_msg = Twist()

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        self.publisher_ = self.create_publisher(Twist, '/yolobot/cmd_vel', 10)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

        topic_name='rgb_cam/image_raw'
        self.cam_publisher_ = self.create_publisher(Image, topic_name, 10)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.cam_callback)


    def timer_callback(self):
        self.publisher_.publish(vel_msg)

    def cam_callback(self):
        self.cam_publisher_.publish(vel_msg)

class Joy_subscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription 

    def listener_callback(self, data):
        vel_msg.linear.x = data.axes[1]
        vel_msg.linear.y = 0.0
        vel_msg.angular.z = data.axes[0]   

if __name__ == '__main__':
    rclpy.init(args=None)
    
    commander = Commander()
    joy_subscriber = Joy_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()

