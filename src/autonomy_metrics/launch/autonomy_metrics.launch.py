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

    # DB resilience / cadence
    db_metrics_period_arg = DeclareLaunchArgument(
        'db_metrics_period',
        default_value='1.0',
        description='Period (s) between periodic distance-and-metrics writes to MongoDB',
    )

    db_server_selection_timeout_ms_arg = DeclareLaunchArgument(
        'db_server_selection_timeout_ms',
        default_value='1000',
        description='MongoDB server-selection timeout (ms) - keeps DB ops from hanging',
    )

    db_connect_timeout_ms_arg = DeclareLaunchArgument(
        'db_connect_timeout_ms',
        default_value='1000',
        description='MongoDB connect timeout (ms)',
    )

    db_socket_timeout_ms_arg = DeclareLaunchArgument(
        'db_socket_timeout_ms',
        default_value='2000',
        description='MongoDB socket timeout (ms)',
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

    # Collision monitor (nav2 collision_detector state)
    collision_detector_min_duration_arg = DeclareLaunchArgument(
        'collision_detector_min_duration',
        default_value='0.0',
        description='Minimum time (s) a detection must persist before an incident is counted',
    )

    collision_detector_clear_time_arg = DeclareLaunchArgument(
        'collision_detector_clear_time',
        default_value='1.0',
        description='Time (s) all zones must stay clear before a new incident can be counted',
    )

    # Odometry robustness
    max_odom_step_distance_arg = DeclareLaunchArgument(
        'max_odom_step_distance',
        default_value='2.5',
        description='Max accepted distance (m) between odom samples; larger steps are treated as jumps',
    )

    max_odom_gap_arg = DeclareLaunchArgument(
        'max_odom_gap',
        default_value='5.0',
        description='Odometry silence (s) after which the pose anchor is reset instead of integrated',
    )

    battery_log_period_arg = DeclareLaunchArgument(
        'battery_log_period',
        default_value='60.0',
        description='Period (s) at which the battery level is checked for a change',
    )

    battery_change_threshold_arg = DeclareLaunchArgument(
        'battery_change_threshold',
        default_value='0.5',
        description='Minimum battery change (%) required to write a new history sample',
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

    db_metrics_period = LaunchConfiguration('db_metrics_period')
    db_server_selection_timeout_ms = LaunchConfiguration('db_server_selection_timeout_ms')
    db_connect_timeout_ms = LaunchConfiguration('db_connect_timeout_ms')
    db_socket_timeout_ms = LaunchConfiguration('db_socket_timeout_ms')

    collision_nav_threshold = LaunchConfiguration('collision_nav_threshold')
    collision_zero_threshold = LaunchConfiguration('collision_zero_threshold')
    collision_time_window = LaunchConfiguration('collision_time_window')
    collision_log_cooldown = LaunchConfiguration('collision_log_cooldown')
    collision_detector_min_duration = LaunchConfiguration('collision_detector_min_duration')
    collision_detector_clear_time = LaunchConfiguration('collision_detector_clear_time')

    max_odom_step_distance = LaunchConfiguration('max_odom_step_distance')
    max_odom_gap = LaunchConfiguration('max_odom_gap')
    battery_log_period = LaunchConfiguration('battery_log_period')
    battery_change_threshold = LaunchConfiguration('battery_change_threshold')

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

            'db_metrics_period': db_metrics_period,
            'db_server_selection_timeout_ms': db_server_selection_timeout_ms,
            'db_connect_timeout_ms': db_connect_timeout_ms,
            'db_socket_timeout_ms': db_socket_timeout_ms,

            'collision_nav_threshold': collision_nav_threshold,
            'collision_zero_threshold': collision_zero_threshold,
            'collision_time_window': collision_time_window,
            'collision_log_cooldown': collision_log_cooldown,
            'collision_detector_min_duration': collision_detector_min_duration,
            'collision_detector_clear_time': collision_detector_clear_time,

            'max_odom_step_distance': max_odom_step_distance,
            'max_odom_gap': max_odom_gap,
            'battery_log_period': battery_log_period,
            'battery_change_threshold': battery_change_threshold,
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
        db_metrics_period_arg,
        db_server_selection_timeout_ms_arg,
        db_connect_timeout_ms_arg,
        db_socket_timeout_ms_arg,
        collision_nav_threshold_arg,
        collision_zero_threshold_arg,
        collision_time_window_arg,
        collision_log_cooldown_arg,
        collision_detector_min_duration_arg,
        collision_detector_clear_time_arg,
        max_odom_step_distance_arg,
        max_odom_gap_arg,
        battery_log_period_arg,
        battery_change_threshold_arg,
        metrics_logger_node,
    ])
