"""
Launch file for CR3 Hand Control Parameter Management System

Starts the parameter manager and diagnostics monitor nodes with proper
configuration and parameter loading.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate the launch description for parameter management."""
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('cr3_hand_control'),
            'config',
            'robot_params.yaml'
        ]),
        description='Path to robot configuration file'
    )
    
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Whether to enable diagnostics monitoring'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level for nodes'
    )
    
    # Parameter Manager Node
    parameter_manager_node = Node(
        package='cr3_hand_control',
        executable='parameter_manager.py',
        name='parameter_manager',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Diagnostics Monitor Node
    diagnostics_monitor_node = Node(
        package='cr3_hand_control',
        executable='diagnostics_monitor.py',
        name='diagnostics_monitor',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        condition=IfCondition(LaunchConfiguration('enable_diagnostics')),
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        config_file_arg,
        enable_diagnostics_arg,
        log_level_arg,
        parameter_manager_node,
        diagnostics_monitor_node,
    ])
