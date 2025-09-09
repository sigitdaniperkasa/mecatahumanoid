from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Launch RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rover'),
                    'launch',
                    'rviz.launch.py'
                )
            )
        ),

        # Teleop
        Node(
            package='rover',
            executable='teleop_rviz.py',
            name='teleop_rviz',
            output='screen'
        ),

        # LIDAR
        Node(
            package='rover',
            executable='fake_lidar.py',
            name='fake_lidar',
            output='screen'
        ),

        # IMU
        Node(
            package='rover',
            executable='fake_imu.py',
            name='fake_imu',
            output='screen'
        ),

        # GPS
        Node(
            package='rover',
            executable='gps_to_tf.py',
            name='gps_to_tf',
            output='screen'
        ),

        # Monitor Motion
        Node(
            package='rover',
            executable='monitor_motion.py',
            name='monitor_motion',
            output='screen'
        )
    ])

