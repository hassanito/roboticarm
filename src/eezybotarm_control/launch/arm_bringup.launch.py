from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('eezybotarm_control')
    
    # URDF file path
    urdf_file = os.path.join(pkg_share, 'description', 'eezybotarm.urdf.xacro')
    
    # RViz config path
    rviz_config = os.path.join(pkg_share, 'config', 'arm.rviz')
    
    # Launch arguments
    update_rate = LaunchConfiguration('update_rate')
    
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'update_rate',
            default_value='10.0',
            description='Update rate for the arm controller'
        )
    ]
    
    # Robot state publisher node
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'publish_frequency': update_rate
        }]
    )
    
    # Joint state publisher node
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'rate': update_rate
        }]
    )
    
    # Arm controller node
    arm_controller = Node(
        package='eezybotarm_control',
        executable='arm_controller',
        name='arm_controller',
        parameters=[{
            'update_rate': update_rate,
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )
    
    # Test patterns node
    test_patterns = Node(
        package='eezybotarm_control',
        executable='arm_test_patterns',
        name='arm_test_patterns'
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription(
        launch_args + [
            robot_state_pub,
            joint_state_pub,
            arm_controller,
            test_patterns,
            rviz_node
        ]
    )
