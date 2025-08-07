#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover')
    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')

    # Generate robot_description from XACRO
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    return LaunchDescription([
        # Publish TFs from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        # (Optional) Joint state publisher GUI if you have movable joints
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Launch RViz with our config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
