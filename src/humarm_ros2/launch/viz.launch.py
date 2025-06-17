from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package paths
    humarm_pkg_share = get_package_share_directory("humarm_ros2")
    
    # Fixed paths for critical files
    xacro_file = os.path.join(humarm_pkg_share, "urdf", "humarmv2.urdf.xacro")
    
    # Generate robot description
    robot_description = {"robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file])}
    
    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    
    # Joint state publisher with GUI for interactive control
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )
    
    # Start RViz with a configuration file (optional)
    # You can create a custom RViz config file for better visualization
    rviz_config_file = os.path.join(humarm_pkg_share, "config", "humanoid_arm.rviz")
    # If the config file doesn't exist, RViz will start with default settings
    rviz_args = []
    if os.path.exists(rviz_config_file):
        rviz_args = ["-d", rviz_config_file]
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=rviz_args,
        parameters=[
            robot_description,
            {"use_sim_time": False}
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])