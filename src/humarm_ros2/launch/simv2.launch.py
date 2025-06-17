from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('humarm_ros2')
    
    # Keep the original file name
    xacro_file = os.path.join(pkg_share, 'urdf', 'humarmv2.urdf.xacro')
    ctrl_config = os.path.join(pkg_share, 'config', 'ros2_control.yaml')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Prepare the robot description from xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ])
    robot_description = {'robot_description': robot_description_content}
    
    return LaunchDescription([
        # Allow sim-time parameter
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Use Gazebo simulation clock'),
            
        # 1) Start Gazebo directly with verbose output
            ExecuteProcess(
                cmd=['killall -9 gzserver gzclient || true', ';', 'sleep', '2', ';', 'gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
                shell=True,  # Use shell to execute the killall command
                output='screen'
            ),
        
        # 2) Wait a bit for Gazebo to start up fully
        TimerAction(
            period=2.0,
            actions=[
                # 3) Start robot_state_publisher
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[robot_description, {'use_sim_time': use_sim_time}]
                ),
                
                # 4) Start controller manager explicitly
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[
                        robot_description,
                        ctrl_config,
                    ],
                    output='screen',
                    remappings=[
                        ('joint_states', '/joint_states'),
                    ],
                ),
            ]
        ),
        
        # 5) Wait a bit longer to make sure controllers are ready
        TimerAction(
            period=5.0,
            actions=[
                # 6) Spawn the robot in Gazebo
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_entity',
                    arguments=[
                        '-entity', 'humanoid_arm',
                        '-topic', 'robot_description',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.05',
                    ],
                    output='screen'
                ),
                
                # 7) Load controllers
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                ),
                
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_trajectory_controller'],
                    output='screen'
                ),
            ]
        ),
    ])