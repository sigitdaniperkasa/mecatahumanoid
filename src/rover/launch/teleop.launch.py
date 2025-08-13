from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('rover')

    robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': PathJoinSubstitution([pkg, 'urdf', 'rover.urdf.xacro'])}]
    )
    jnt_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    # ros2_control controller manager
    controller = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[PathJoinSubstitution([pkg, 'config', 'ros2_control.yaml']),
                    {'robot_description': PathJoinSubstitution([pkg, 'urdf', 'rover.urdf.xacro'])}]
    )
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )
    spawn_dd  = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller']
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg, 'config', 'robot.rviz'])]
    )
    
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_state,
        jnt_pub,
        controller,
        spawn_jsb,
        spawn_dd,
        rviz,
        teleop,              # now as a Node
    ])

