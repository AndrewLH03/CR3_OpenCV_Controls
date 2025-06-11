#!/usr/bin/env python3
"""
Launch file for Complete Phase 3 System
Launches all Phase 3 components: safety system and coordinate transformations.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for complete Phase 3 system"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('cr3_hand_control')
    
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
    
    # Include safety system launch
    safety_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cr3_hand_control'),
                'launch',
                'subsystems',
                'safety_system.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level')
        }.items()
    )
    
    # Include coordinate system launch
    coordinate_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cr3_hand_control'),
                'launch',
                'subsystems',
                'coordinate_system.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'run_calibration': LaunchConfiguration('run_calibration'),
            'log_level': LaunchConfiguration('log_level')
        }.items()
    )
    
    # Basic robot controller from Phase 2
    robot_controller_node = Node(
        package='cr3_hand_control',
        executable='basic_robot_controller',
        name='basic_robot_controller',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'robot', 'cr3_parameters.yaml'),
            os.path.join(pkg_dir, 'config', 'robot', 'motion_config.yaml'),
            os.path.join(pkg_dir, 'config', 'safety', 'safety_params_ros2.yaml')
        ]
    )
    
    # Parameter manager from Phase 2
    parameter_manager_node = Node(
        package='cr3_hand_control',
        executable='parameter_manager.py',
        name='parameter_manager',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'robot', 'cr3_parameters.yaml')
        ]
    )
    
    # Diagnostics monitor from Phase 2
    diagnostics_monitor_node = Node(
        package='cr3_hand_control',
        executable='diagnostics_monitor.py',
        name='diagnostics_monitor',
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_run_calibration,
        declare_log_level,
        
        # Phase 2 components
        GroupAction([
            robot_controller_node,
            parameter_manager_node,
            diagnostics_monitor_node
        ], scoped=False),
        
        # Phase 3 components
        safety_system_launch,
        coordinate_system_launch
    ])
