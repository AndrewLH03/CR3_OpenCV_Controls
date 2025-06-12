#!/usr/bin/env python3
"""
Launch file for pose recognition subsystem
Based on working Dashboards implementation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera device ID'
    )
    
    tracked_hand_arg = DeclareLaunchArgument(
        'tracked_hand',
        default_value='Right',
        description='Which hand to track (Right or Left)'
    )
    
    enable_debug_arg = DeclareLaunchArgument(
        'enable_debug_image',
        default_value='true',
        description='Enable debug image visualization'
    )
    
    enable_robot_arg = DeclareLaunchArgument(
        'enable_robot_control',
        default_value='true',
        description='Enable robot control integration'
    )
    
    # Get package directory
    package_dir = FindPackageShare('cr3_hand_control')
    
    # Configuration file path
    config_file = PathJoinSubstitution([
        package_dir,
        'config',
        'perception',
        'pose_recognition_params.yaml'
    ])
    
    # Pose recognition node
    pose_recognition_node = Node(
        package='cr3_hand_control',
        executable='pose_recognition_node',
        name='pose_recognition_node',
        namespace='perception',
        parameters=[
            config_file,
            {
                'camera_id': LaunchConfiguration('camera_id'),
                'tracked_hand': LaunchConfiguration('tracked_hand'),
                'enable_debug_image': LaunchConfiguration('enable_debug_image'),
                'enable_robot_control': LaunchConfiguration('enable_robot_control'),
            }
        ],
        remappings=[
            ('pose_coordinates', '/perception/pose_coordinates'),
            ('pose_tracking_status', '/perception/pose_tracking_status'),
            ('pose_debug_info', '/perception/pose_debug_info'),
            ('pose_debug_image', '/perception/pose_debug_image'),
            ('robot_enabled', '/robot/enabled'),
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        LogInfo(msg="Starting Pose Recognition Subsystem"),
        camera_id_arg,
        tracked_hand_arg,
        enable_debug_arg,
        enable_robot_arg,
        pose_recognition_node,
        LogInfo(msg="Pose Recognition Subsystem started successfully"),
    ])
