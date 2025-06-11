#!/usr/bin/env python3
"""
Launch file for Perception Subsystem
Launches camera and hand tracking nodes for the CR3 robot system.
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
    camera_config_file = os.path.join(pkg_dir, 'config', 'perception', 'camera_config.yaml')
    hand_tracking_config_file = os.path.join(pkg_dir, 'config', 'perception', 'hand_tracking_config.yaml')
    
    # Camera Node (Future implementation)
    # camera_node = Node(
    #     package='cr3_hand_control',
    #     executable='camera_node.py',
    #     name='camera_node',
    #     output='screen',
    #     parameters=[camera_config_file],
    #     remappings=[
    #         ('/camera/image_raw', '/camera/image_raw'),
    #         ('/camera/camera_info', '/camera/camera_info')
    #     ]
    # )
    
    # Hand Tracking Node (Future implementation)
    # hand_tracking_node = Node(
    #     package='cr3_hand_control',
    #     executable='hand_tracking_node.py',
    #     name='hand_tracking_node',
    #     output='screen',
    #     parameters=[hand_tracking_config_file],
    #     remappings=[
    #         ('/camera/image_raw', '/camera/image_raw'),
    #         ('/hand_position', '/hand_position'),
    #         ('/hand_landmarks', '/hand_landmarks')
    #     ]
    # )
    
    # Placeholder node for future perception system
    placeholder_node = Node(
        package='cr3_hand_control',
        executable='parameter_manager.py',  # Using existing node as placeholder
        name='perception_placeholder',
        output='screen',
        parameters=[{
            'perception_system_ready': False,
            'planned_for_phase': 4
        }]
    )
    
    return [
        placeholder_node
        # camera_node,    # Uncomment when implemented
        # hand_tracking_node  # Uncomment when implemented
    ]


def generate_launch_description():
    """Generate launch description for perception system"""
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_enable_camera = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='Enable camera node if true'
    )
    
    declare_enable_hand_tracking = DeclareLaunchArgument(
        'enable_hand_tracking',
        default_value='false',
        description='Enable hand tracking node if true'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_camera,
        declare_enable_hand_tracking,
        declare_log_level,
        OpaqueFunction(function=launch_setup)
    ])
