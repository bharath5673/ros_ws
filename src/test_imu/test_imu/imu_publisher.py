#! /usr/bin/env python3

import rclpy
import random
from sensor_msgs.msg import Imu

class ImuPublisher:
    def __init__(self):


        self.topic_name = 'imu'
        self.frame_id_ = 'map'

        self.node = rclpy.create_node('imu_publisher')
        self.pub = self.node.create_publisher(Imu, self.topic_name, 10)
        self.timer = self.node.create_timer(1.0, self.publish_imu)
        
    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.frame_id = self.frame_id_   # Set the frame_id of the message
        imu_msg.orientation.x = random.uniform(0.001, 20.00)
        imu_msg.orientation.y = random.uniform(0.001, 20.00)
        imu_msg.orientation.z = random.uniform(0.001, 20.00)
        imu_msg.angular_velocity.x = random.uniform(0.001, 20.00)
        imu_msg.angular_velocity.y = random.uniform(0.001, 20.00)
        imu_msg.angular_velocity.z = random.uniform(0.001, 20.00)
        imu_msg.linear_acceleration.x = random.uniform(0.001, 20.00)
        imu_msg.linear_acceleration.y = random.uniform(0.001, 20.00)
        imu_msg.linear_acceleration.z = random.uniform(0.001, 20.00)
        imu_msg.orientation_covariance = [random.uniform(0.001, 20.00) for _ in range(9)]
        imu_msg.angular_velocity_covariance = [random.uniform(0.001, 20.00) for _ in range(9)]
        imu_msg.linear_acceleration_covariance = [random.uniform(0.001, 20.00) for _ in range(9)]
        print(imu_msg)
        self.pub.publish(imu_msg)

def main():
    rclpy.init()
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher.node)
    imu_publisher.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()