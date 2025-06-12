#!/usr/bin/env python3
"""
Test launch file for pose recognition only
For development and debugging of pose recognition functionality
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


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
    
    show_image_arg = DeclareLaunchArgument(
        'show_image',
        default_value='true',
        description='Show debug image window'
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
    
    # Pose recognition node with testing parameters
    pose_recognition_node = Node(
        package='cr3_hand_control',
        executable='pose_recognition_node',
        name='pose_recognition_test_node',
        parameters=[
            config_file,
            {
                'camera_id': LaunchConfiguration('camera_id'),
                'tracked_hand': LaunchConfiguration('tracked_hand'),
                'enable_debug_image': True,
                'enable_robot_control': False,  # Disabled for testing
                'publish_rate': 30.0,
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # RViz for visualization (optional)
    rviz_config = PathJoinSubstitution([
        package_dir,
        'config',
        'rviz',
        'pose_recognition_test.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='pose_recognition_rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('show_image')),
        output='screen'
    )
    
    # Image view for debug visualization
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='pose_debug_image_viewer',
        arguments=['/pose_debug_image'],
        condition=IfCondition(LaunchConfiguration('show_image')),
        output='screen'
    )
    
    return LaunchDescription([
        LogInfo(msg="Starting Pose Recognition Test"),
        camera_id_arg,
        tracked_hand_arg,
        show_image_arg,
        pose_recognition_node,
        # Uncomment these when testing visualization
        # rviz_node,
        # image_view_node,
        LogInfo(msg="Pose Recognition Test started - Press Ctrl+C to stop"),
    ])
