import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

def generate_launch_description():
    # Path to your URDF file
    pkg_share = get_package_share_directory("torso")
    xacro_file = os.path.join(pkg_share, "urdf", "torso.urdf.xacro")

    # Generate robot description
    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file])
    }

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]     
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'torso_rviz.rviz')]
    )

    # Controller Manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(pkg_share, 'config', 'ros2_control.yaml')],
        output='screen'
    )

    # Load controllers
    def load_controller(controller_name):
        return ExecuteProcess(
            cmd=['ros2 run controller_manager spawner.py ' + controller_name],
            shell=True,
            output='screen'
        )

    return LaunchDescription([
        # Start robot state publisher
        robot_state_publisher,
        
        # Start joint state publisher GUI for manual control
        joint_state_publisher_gui,
        
        # Start RViz
        rviz_node,
        
        # Start controller manager
        controller_manager,
        
        # Load controllers
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_manager,
                on_exit=[load_controller('joint_state_broadcaster')]
        )),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_manager,
                on_exit=[load_controller('torso_controller')]
        )),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_manager,
                on_exit=[load_controller('neck_controller')]
        )),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_manager,
                on_exit=[load_controller('right_arm_controller')]
        )),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_manager,
                on_exit=[load_controller('left_arm_controller')]
        )),
    ])