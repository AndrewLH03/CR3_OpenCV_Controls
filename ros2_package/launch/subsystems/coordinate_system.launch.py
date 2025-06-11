#!/usr/bin/env python3
"""
Launch file for Phase 3 Coordinate System
Launches all coordinate transformation nodes for the CR3 robot system.
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
    calibration_params_file = os.path.join(pkg_dir, 'config', 'coordination', 'calibration_params.yaml')
    transform_config_file = os.path.join(pkg_dir, 'config', 'coordination', 'transform_config.yaml')
    frame_management_file = os.path.join(pkg_dir, 'config', 'coordination', 'frame_management.yaml')
    robot_params_file = os.path.join(pkg_dir, 'config', 'robot', 'cr3_parameters.yaml')
    
    # Transform Manager Node (Python)
    transform_manager_node = Node(
        package='cr3_hand_control',
        executable='transform_manager.py',
        name='transform_manager',
        output='screen',
        parameters=[transform_config_file, robot_params_file],
        remappings=[
            ('/coordinate_transforms', '/coordinate_transforms')
        ]
    )
    
    # Coordinate Broadcaster Node (C++)
    coordinate_broadcaster_node = Node(
        package='cr3_hand_control',
        executable='coordinate_broadcaster',
        name='coordinate_broadcaster',
        output='screen',
        parameters=[frame_management_file, calibration_params_file],
    )
    
    # Calibration Node (Python) - Optional, can be launched separately
    from launch.conditions import IfCondition
    
    calibration_node = Node(
        package='cr3_hand_control',
        executable='calibration_node.py',
        name='calibration_node',
        output='screen',
        parameters=[calibration_params_file, robot_params_file],
        remappings=[
            ('/robot_status', '/robot_status'),
            ('/calibration_commands', '/calibration_commands'),
            ('/calibration_transforms', '/calibration_transforms')
        ],
        condition=IfCondition(LaunchConfiguration('run_calibration'))
    )
    
    return [
        transform_manager_node,
        coordinate_broadcaster_node,
        calibration_node
    ]


def generate_launch_description():
    """Generate launch description for coordinate system"""
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_run_calibration = DeclareLaunchArgument(
        'run_calibration',
        default_value='false',
        description='Run calibration node if true'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_run_calibration,
        declare_log_level,
        OpaqueFunction(function=launch_setup)
    ])
