#!/usr/bin/env python3
"""
Launch file for Phase 3 Safety System
Launches all safety-related nodes for the CR3 robot system.
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
    safety_params_file = os.path.join(pkg_dir, 'config', 'safety_params.yaml')
    robot_params_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
    
    # Safety Monitor Node (Python)
    safety_monitor_node = Node(
        package='cr3_hand_control',
        executable='safety_monitor.py',
        name='safety_monitor',
        output='screen',
        parameters=[safety_params_file, robot_params_file],
        remappings=[
            ('/robot_status', '/robot_status'),
            ('/safety_alerts', '/safety_alerts'),
            ('/safety_commands', '/safety_commands'),
            ('/emergency_stop', '/emergency_stop')
        ]
    )
    
    # Workspace Validator Node (C++)
    workspace_validator_node = Node(
        package='cr3_hand_control',
        executable='workspace_validator',
        name='workspace_validator',
        output='screen',
        parameters=[safety_params_file, robot_params_file],
        remappings=[
            ('/robot_status', '/robot_status'),
            ('/workspace_valid', '/workspace_valid')
        ]
    )
    
    # Emergency Stop Handler Node (C++)
    emergency_stop_node = Node(
        package='cr3_hand_control',
        executable='emergency_stop_handler',
        name='emergency_stop_handler',
        output='screen',
        parameters=[safety_params_file],
        remappings=[
            ('/safety_alerts', '/safety_alerts'),
            ('/robot_status', '/robot_status'),
            ('/emergency_commands', '/emergency_commands'),
            ('/emergency_active', '/emergency_active'),
            ('/emergency_stop_handler', '/emergency_stop_handler')
        ]
    )
    
    return [
        safety_monitor_node,
        workspace_validator_node,
        emergency_stop_node
    ]


def generate_launch_description():
    """Generate launch description for safety system"""
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_log_level,
        OpaqueFunction(function=launch_setup)
    ])
