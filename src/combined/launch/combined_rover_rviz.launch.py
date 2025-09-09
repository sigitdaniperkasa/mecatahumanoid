from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('rover')
    urdf_file = os.path.join(pkg_share, 'urdf', 'combined_rover.urdf')

    return LaunchDescription([
        # Robot State Publisher (baca URDF dan broadcast ke TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_file],
            output='screen'
        ),

        # Joint State Publisher GUI (biar bisa gerakin torso/tangan manual)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
