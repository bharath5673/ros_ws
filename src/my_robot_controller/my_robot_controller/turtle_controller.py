#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from turtlesim.srv import SetPen
from functools import partial


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        ##start publisher
        self.cmd_vel_publiser_ = self.create_publisher(Twist , 'turtle1/cmd_vel', 10)
        self.get_logger().info('started publisher')

        ##start subscriber
        self.pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.pose_callback, 10)
        self.get_logger().info('started subscriber')
        
        ##notice
        self.get_logger().info('Turtle coltroller has been started.')


    ##enforcing turtlebot using callback for turtlecontrollernode
    def pose_callback(self, pose: Pose):
        # pass or  
  
        cmd = Twist()
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0 :
            cmd.linear.x = 1.0
            cmd.angular.z = 0.8
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_publiser_.publish(cmd)

  
        ###setpen_service
        if pose.x > 5.5:
            self.get_logger().info('setting color to red')
            self.call_set_pen_service(255, 0, 0, 3, 0)
        else:
            self.get_logger().info('setting color to green')
            self.call_set_pen_service(0, 255, 0, 3, 0)      


    ##service backend
    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn('waiting for service... ')

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.setpen_callback))

    def setpen_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('service call failed: %r' %(e,))
        pass 


def main(args=None): 
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown() 