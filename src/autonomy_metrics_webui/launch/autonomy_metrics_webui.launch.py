#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("ui_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("ui_port", default_value="8081"),
            DeclareLaunchArgument("ui_access_log", default_value="false"),
            DeclareLaunchArgument("mongodb_host", default_value="localhost"),
            DeclareLaunchArgument("mongodb_port", default_value="27018"),
            DeclareLaunchArgument("remote_mongodb_host", default_value=""),
            DeclareLaunchArgument("remote_mongodb_port", default_value="27017"),
            DeclareLaunchArgument("prefer_remote_data", default_value="false"),
            DeclareLaunchArgument("database_name", default_value="robot_incidents"),
            Node(
                package="autonomy_metrics_webui",
                executable="autonomy_metrics_webui_server",
                name="autonomy_metrics_webui_server",
                output="screen",
                parameters=[
                    {
                        "ui_host": LaunchConfiguration("ui_host"),
                        "ui_port": LaunchConfiguration("ui_port"),
                        "ui_access_log": LaunchConfiguration("ui_access_log"),
                        "mongodb_host": LaunchConfiguration("mongodb_host"),
                        "mongodb_port": LaunchConfiguration("mongodb_port"),
                        "remote_mongodb_host": LaunchConfiguration("remote_mongodb_host"),
                        "remote_mongodb_port": LaunchConfiguration("remote_mongodb_port"),
                        "prefer_remote_data": LaunchConfiguration("prefer_remote_data"),
                        "database_name": LaunchConfiguration("database_name"),
                    }
                ],
            ),
        ]
    )
