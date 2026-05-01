#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("robmini_description")

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robmini"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("sim_drive_mode", default_value="ros2_control", choices=["ros2_control", "planar"]),
        DeclareLaunchArgument("world_name", default_value=os.path.join(pkg_share, "worlds", "room_mini.world")),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("debug", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "sim_02_launch.py")),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "namespace": LaunchConfiguration("namespace"),
                "tf_prefix": LaunchConfiguration("tf_prefix"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "sim_drive_mode": LaunchConfiguration("sim_drive_mode"),
                "world": LaunchConfiguration("world_name"),
                "gui": LaunchConfiguration("gui"),
                "paused": LaunchConfiguration("paused"),
                "headless": LaunchConfiguration("headless"),
                "debug": LaunchConfiguration("debug"),
            }.items(),
        ),
    ])
