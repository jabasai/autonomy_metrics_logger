#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # -------------------------------------------------------------------------
    # Launch arguments (all node parameters exposed)
    # -------------------------------------------------------------------------
    
    pkg_dir = get_package_share_directory('autonomy_metrics')
    
    config_yaml_arg = DeclareLaunchArgument(
        'config_yaml',
        default_value=os.path.join(pkg_dir, 'config', 'metrics_full.yaml'),
        description='Path to the YAML config file for AutonomyMetricsLogger',
    )

    mongodb_host_arg = DeclareLaunchArgument(
        'mongodb_host',
        default_value='localhost',
        description='Hostname of local MongoDB instance',
    )

    mongodb_port_arg = DeclareLaunchArgument(
        'mongodb_port',
        default_value='27018',
        description='Port of local MongoDB instance',
    )

    remote_mongodb_host_arg = DeclareLaunchArgument(
        'remote_mongodb_host',
        default_value='',
        description='Hostname of remote MongoDB instance (leave empty to disable)',
    )

    remote_mongodb_port_arg = DeclareLaunchArgument(
        'remote_mongodb_port',
        default_value='27017',
        description='Port of remote MongoDB instance',
    )

    enable_remote_logging_arg = DeclareLaunchArgument(
        'enable_remote_logging',
        default_value='false',
        description='Enable logging to remote MongoDB (true/false)',
    )

    min_distance_threshold_arg = DeclareLaunchArgument(
        'min_distance_threshold',
        default_value='0.2',
        description='Minimum odom distance increment (m) required to update metrics',
    )

    stop_timeout_arg = DeclareLaunchArgument(
        'stop_timeout',
        default_value='2.0',
        description='Timeout (s) after last odom movement before speed is forced to 0',
    )

    # Collision monitor (nav vs collision output)
    collision_nav_threshold_arg = DeclareLaunchArgument(
        'collision_nav_threshold',
        default_value='0.01',
        description='Minimum nav linear.x to consider a forward motion command',
    )

    collision_zero_threshold_arg = DeclareLaunchArgument(
        'collision_zero_threshold',
        default_value='0.001',
        description='Absolute linear.x threshold below which collision output is considered zero',
    )

    collision_time_window_arg = DeclareLaunchArgument(
        'collision_time_window',
        default_value='0.5',
        description='Max age (s) of nav/collision commands to consider for collision detection',
    )

    collision_log_cooldown_arg = DeclareLaunchArgument(
        'collision_log_cooldown',
        default_value='1.0',
        description='Cooldown (s) between consecutive collision logs (if you still use cooldown)',
    )

    # -------------------------------------------------------------------------
    # LaunchConfigurations (bind arguments to parameters)
    # -------------------------------------------------------------------------
    config_yaml = LaunchConfiguration('config_yaml')
    mongodb_host = LaunchConfiguration('mongodb_host')
    mongodb_port = LaunchConfiguration('mongodb_port')
    remote_mongodb_host = LaunchConfiguration('remote_mongodb_host')
    remote_mongodb_port = LaunchConfiguration('remote_mongodb_port')
    enable_remote_logging = LaunchConfiguration('enable_remote_logging')
    min_distance_threshold = LaunchConfiguration('min_distance_threshold')
    stop_timeout = LaunchConfiguration('stop_timeout')

    collision_nav_threshold = LaunchConfiguration('collision_nav_threshold')
    collision_zero_threshold = LaunchConfiguration('collision_zero_threshold')
    collision_time_window = LaunchConfiguration('collision_time_window')
    collision_log_cooldown = LaunchConfiguration('collision_log_cooldown')

    # -------------------------------------------------------------------------
    # AutonomyMetricsLogger node
    # -------------------------------------------------------------------------
    metrics_logger_node = Node(
        package='autonomy_metrics',
        executable='metric_logger',
        name='mdbi_logger_dynamic',
        output='screen',
        parameters=[{
            'config_yaml': config_yaml,
            'mongodb_host': mongodb_host,
            'mongodb_port': mongodb_port,
            'remote_mongodb_host': remote_mongodb_host,
            'remote_mongodb_port': remote_mongodb_port,
            'enable_remote_logging': enable_remote_logging,
            'min_distance_threshold': min_distance_threshold,
            'stop_timeout': stop_timeout,

            'collision_nav_threshold': collision_nav_threshold,
            'collision_zero_threshold': collision_zero_threshold,
            'collision_time_window': collision_time_window,
            'collision_log_cooldown': collision_log_cooldown,
        }],
    )

    return LaunchDescription([
        config_yaml_arg,
        mongodb_host_arg,
        mongodb_port_arg,
        remote_mongodb_host_arg,
        remote_mongodb_port_arg,
        enable_remote_logging_arg,
        min_distance_threshold_arg,
        stop_timeout_arg,
        collision_nav_threshold_arg,
        collision_zero_threshold_arg,
        collision_time_window_arg,
        collision_log_cooldown_arg,
        metrics_logger_node,
    ])
