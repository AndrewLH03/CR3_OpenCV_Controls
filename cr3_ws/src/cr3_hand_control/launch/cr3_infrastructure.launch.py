#!/usr/bin/env python3
"""
Launch file for CR3 Hand Control Phase 2 Infrastructure

Launches the core infrastructure components:
- Parameter Manager
- Diagnostics Monitor
- Basic Robot Controller (when ready)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('cr3_hand_control'),
            'config',
            'robot_params.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    start_diagnostics_arg = DeclareLaunchArgument(
        'start_diagnostics',
        default_value='true',
        description='Whether to start diagnostics monitor'
    )
    
    start_robot_controller_arg = DeclareLaunchArgument(
        'start_robot_controller',
        default_value='false',
        description='Whether to start basic robot controller'
    )
    
    # Parameter Manager Node
    parameter_manager_node = Node(
        package='cr3_hand_control',
        executable='parameter_manager',
        name='parameter_manager',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    # Diagnostics Monitor Node
    diagnostics_monitor_node = Node(
        package='cr3_hand_control',
        executable='diagnostics_monitor',
        name='diagnostics_monitor',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_diagnostics'))
    )
    
    # Basic Robot Controller Node (C++)
    basic_robot_controller_node = Node(
        package='cr3_hand_control',
        executable='basic_robot_controller',
        name='basic_robot_controller',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_robot_controller'))
    )
    
    return LaunchDescription([
        config_file_arg,
        start_diagnostics_arg,
        start_robot_controller_arg,
        parameter_manager_node,
        diagnostics_monitor_node,
        basic_robot_controller_node,
    ])
