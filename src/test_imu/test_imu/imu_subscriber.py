#! /usr/bin/env python3

import rclpy
from sensor_msgs.msg import Imu


class ImuSubscriber:
    def __init__(self):
        self.node = rclpy.create_node('imu_subscriber')
        self.topic_name = 'imu'
        self.sub = self.node.create_subscription(Imu, self.topic_name, imu_callback, 10)


def imu_callback(imu_msg):
    print('Received IMU orientation: ({}, {}, {})'.format(imu_msg.orientation.x,
                                                           imu_msg.orientation.y,
                                                           imu_msg.orientation.z))
    print('Received IMU angular velocity: ({}, {}, {})'.format(imu_msg.angular_velocity.x,
                                                                imu_msg.angular_velocity.y,
                                                                imu_msg.angular_velocity.z))
    print('Received IMU linear acceleration: ({}, {}, {})'.format(imu_msg.linear_acceleration.x,
                                                                   imu_msg.linear_acceleration.y,
                                                                   imu_msg.linear_acceleration.z))
    print('Received IMU orientation covariance: {}'.format(imu_msg.orientation_covariance))
    print('Received IMU angular velocity covariance: {}'.format(imu_msg.angular_velocity_covariance))
    print('Received IMU linear acceleration covariance: {}'.format(imu_msg.linear_acceleration_covariance))
    

def main():
    rclpy.init()
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber.node)
    imu_subscriber.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()