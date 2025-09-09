#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover')
    xacro_path = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', xacro_path])}

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             output='screen'),
        Node(package='rviz2', executable='rviz2', output='screen'),
        Node(package='rover', executable='teleop_rviz.py', output='screen'),
    ])
