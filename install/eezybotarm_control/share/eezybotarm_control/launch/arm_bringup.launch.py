from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start arm controller
        Node(
            package='eezybotarm_control',
            executable='arm_controller',
            name='arm_controller',
            parameters=[
                {'update_rate': 10.0}
            ]
        ),
        # Start test patterns
        Node(
            package='eezybotarm_control',
            executable='arm_test_patterns',
            name='arm_test_patterns'
        )
    ])