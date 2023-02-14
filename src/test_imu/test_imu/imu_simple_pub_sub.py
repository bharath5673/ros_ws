#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import random

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.topic_name = 'imu'
        self.frame_id_ = 'map'

        self.publisher = self.create_publisher(Imu, self.topic_name, 10)
        self.timer = self.create_timer(0.1, self.publish_imu)

    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.orientation.x = random.uniform(-1, 1)
        imu_msg.orientation.y = random.uniform(-1, 1)
        imu_msg.orientation.z = random.uniform(-1, 1)
        # imu_msg.orientation.w = random.uniform(-20, 20)
        imu_msg.header.frame_id = self.frame_id_  # set the frame ID
        self.publisher.publish(imu_msg)
        self.get_logger().info(f"Published IMU orientation: {imu_msg.orientation}")

    def subscribe_imu(self, msg):
        self.get_logger().info(f"Received IMU orientation: {msg.orientation}")

def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuNode()
    subscriber = imu_node.create_subscription(Imu, 'imu', imu_node.subscribe_imu, 10)
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

