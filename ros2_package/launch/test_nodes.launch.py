from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cr3_hand_control',
            executable='basic_robot_controller',
            name='basic_robot_controller',
            output='screen'
        ),
        Node(
            package='cr3_hand_control',
            executable='diagnostics_monitor.py',
            name='diagnostics_monitor',
            output='screen'
        )
    ])
