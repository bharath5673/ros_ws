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
        self.writer = rosbag2_py.SequentialWriter()

        ###rosbaging
        currentDateAndTime = datetime.now()
        currentTime = currentDateAndTime.strftime("%Y_%m_%d-%H_%M_%S")
        bag_name = 'rosbag2_'+str(currentTime)
        self.bag_path= 'ros_ws/src/test_turtle_bag/test_turtle_bag/'+bag_name

        ##notice
        self.get_logger().info('saving bag file at : '+ self.bag_path)

        storage_options = rosbag2_py._storage.StorageOptions(uri=self.bag_path,storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        self.topic_name='/turtle1/cmd_vel'
        self.type_name='geometry_msgs/msg/Twist'

        topic_info = rosbag2_py._storage.TopicMetadata(name=self.topic_name,type=self.type_name,serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.subscription = self.create_subscription(Twist,self.topic_name,self.topic_callback,10)
        self.subscription

    def topic_callback(self, msg):
        self.writer.write(self.topic_name ,serialize_message(msg),self.get_clock().now().nanoseconds)

    

def main(args=None): 
    rclpy.init(args=args)
    node = TurtleBagNode()
    rclpy.spin(node)
    rclpy.shutdown() 