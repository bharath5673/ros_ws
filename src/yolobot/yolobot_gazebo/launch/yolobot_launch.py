#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_yolobot = get_package_share_directory('yolobot')
    # pkg_yolobot_description = get_package_share_directory('yolobot')
    # pkg_yolobot_control = get_package_share_directory('yolobot')

    pkg_yolobot_gazebo = 'ros_ws/src/yolobot/yolobot_gazebo'
    pkg_yolobot_description = 'ros_ws/src/yolobot/yolobot_description'
    pkg_yolobot_control = 'ros_ws/src/yolobot/yolobot_control'



    joy_node = Node(
        package = "joy",
        executable = "joy_node"
    )

    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_gazebo, 'launch', 'start_world_launch.py'),
        )
    )

    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_description, 'launch', 'spawn_yolobot_launch.launch.py'),
        )
    )     

    spawn_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_control, 'launch', 'yolobot_control.launch.py'),
        )
    )  

    return LaunchDescription([
        joy_node,
        start_world,
        spawn_robot_world,
        spawn_robot_control,
    ])