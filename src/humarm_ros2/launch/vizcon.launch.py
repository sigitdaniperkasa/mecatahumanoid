from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package paths
    humarm_pkg_share = get_package_share_directory("humarm_ros2")
    
    # Launch arguments
    can_interface = LaunchConfiguration('can_interface')
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface to use (e.g., can0)'
    )
    
    # Fixed paths for critical files
    xacro_file = os.path.join(humarm_pkg_share, "urdf", "humarmv2.urdf.xacro")
    # Use absolute path to the script to avoid any path resolution issues
    bridge_script_path = os.path.join(
        os.path.expanduser('~'),  # Home directory
        'vm/humarm/src/humarm_ros2/scripts/joint2can.py'  # Script path
    )
    
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
    
    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )
    
    motor_bridge = ExecuteProcess(
        cmd=['python3', bridge_script_path],
        name='joint_to_can',
        output='screen',
    )
    
    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": False}
        ]
    )
    
    return LaunchDescription([
        can_interface_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        motor_bridge,
        rviz_node
    ])