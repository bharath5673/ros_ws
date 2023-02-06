#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from rclpy.serialization import serialize_message
from std_msgs.msg import String

import rosbag2_py
from datetime import datetime

class TurtleBagNode(Node):
    def __init__(self):
        super().__init__('simple_bag_recorder')

        ##start publisher
        self.cmd_vel_publiser_ = self.create_publisher(Twist , 'turtle1/cmd_vel', 10)
        self.get_logger().info('started publisher')

        ##start subscriber
        self.pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.pose_callback, 10)
        self.get_logger().info('started subscriber')
        
        ##notice
        self.get_logger().info('Turtle coltroller has been started.')
        self.get_logger().info('rosbag has been started...')


        ###rosbaging
        currentDateAndTime = datetime.now()
        currentTime = currentDateAndTime.strftime("%Y_%m_%d-%H_%M_%S")
        bag_name = 'rosbag2_'+str(currentTime)

        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(uri='ros_ws/src/test_turtle_bag/test_turtle_bag/'+bag_name,storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(name='Twist',type='std_msgs/msg/String',serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.subscription = self.create_subscription(Pose, "turtle1/pose",self.topic_callback,10)
        self.subscription

    def topic_callback(self, msg):
        self.writer.write('Twist',serialize_message(msg),self.get_clock().now().nanoseconds)



    ##enforcing turtlebot using callback for turtlecontrollernode
    def pose_callback(self, pose: Pose):
        # pass or  
        cmd = Twist()
        self.cmd_vel_publiser_.publish(cmd)

  


def main(args=None): 
    rclpy.init(args=args)
    node = TurtleBagNode()
    rclpy.spin(node)
    rclpy.shutdown() 