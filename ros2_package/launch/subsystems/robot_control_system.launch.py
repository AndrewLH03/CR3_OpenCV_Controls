#!/usr/bin/env python3
"""
Launch file for Robot Control Subsystem
Launches core robot control nodes for the CR3 robot system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """Setup function to configure launch with parameters"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('cr3_hand_control')
    
    # Config file paths
    robot_params_file = os.path.join(pkg_dir, 'config', 'robot', 'cr3_parameters.yaml')
    motion_config_file = os.path.join(pkg_dir, 'config', 'robot', 'motion_config.yaml')
    
    # Basic Robot Controller Node (C++)
    robot_controller_node = Node(
        package='cr3_hand_control',
        executable='basic_robot_controller',
        name='basic_robot_controller',
        output='screen',
        parameters=[robot_params_file, motion_config_file],
        remappings=[
            ('/robot_commands', '/robot_commands'),
            ('/robot_status', '/robot_status'),
            ('/joint_states', '/joint_states')
        ]
    )
    
    # Parameter Manager Node (Python)
    parameter_manager_node = Node(
        package='cr3_hand_control',
        executable='parameter_manager.py',
        name='parameter_manager',
        output='screen',
        parameters=[robot_params_file],
        remappings=[
            ('/parameter_updates', '/parameter_updates'),
            ('/robot_config', '/robot_config')
        ]
    )
    
    # Diagnostics Monitor Node (Python)
    diagnostics_monitor_node = Node(
        package='cr3_hand_control',
        executable='diagnostics_monitor.py',
        name='diagnostics_monitor',
        output='screen',
        parameters=[robot_params_file],
        remappings=[
            ('/robot_status', '/robot_status'),
            ('/diagnostics', '/diagnostics'),
            ('/system_health', '/system_health')
        ]
    )
    
    return [
        robot_controller_node,
        parameter_manager_node,
        diagnostics_monitor_node
    ]


def generate_launch_description():
    """Generate launch description for robot control system"""
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_enable_diagnostics = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Enable diagnostics monitoring if true'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_diagnostics,
        declare_log_level,
        OpaqueFunction(function=launch_setup)
    ])
