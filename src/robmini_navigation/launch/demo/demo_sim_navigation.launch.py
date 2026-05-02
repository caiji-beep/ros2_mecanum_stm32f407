#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    namespace = LaunchConfiguration("namespace")
    tf_prefix = LaunchConfiguration("tf_prefix")
    map_frame = LaunchConfiguration("map_frame")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_drive_mode = LaunchConfiguration("sim_drive_mode")
    use_rviz = LaunchConfiguration("use_rviz")
    map_file = LaunchConfiguration("map_file")

    pkg_description = get_package_share_directory("robmini_description")
    pkg_navigation = get_package_share_directory("robmini_navigation")

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, "launch", "sim_02_launch.py")
        ),
        launch_arguments={
            "robot_name": robot_name,
            "namespace": namespace,
            "tf_prefix": tf_prefix,
            "use_sim_time": use_sim_time,
            "sim_drive_mode": sim_drive_mode,
        }.items(),
    )

    nav_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "robot_bringup.launch.py")
        ),
        launch_arguments={
            "robot_name": robot_name,
            "namespace": namespace,
            "tf_prefix": tf_prefix,
            "map_frame": map_frame,
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
            "map_file": map_file
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robmini"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("map_frame", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("map_file",default_value="room_mini/room_mini.yaml"),
        DeclareLaunchArgument("sim_drive_mode", default_value="planar", choices=["ros2_control", "planar"]),
        simulation_launch,
        TimerAction(period=12.0, actions=[nav_bringup_launch]),
    ])
