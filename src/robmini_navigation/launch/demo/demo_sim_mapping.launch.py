#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file demo_sim_mapping.launch.py
@brief RobMini 实机建图总启动文件

该 launch 文件用于按顺序启动 RobMini 实机建图所需模块：
1. sim_02.launch.py：机器人本体、ros2_control、状态发布等；
2. bringup_mapping_sim.launch.py：slam_toolbox 建图和可选 RViz。

启动顺序通过 TimerAction 控制，避免 SLAM 在底盘、TF 或雷达尚未就绪时提前启动。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    @brief 生成 RobMini 实机建图系统的 launch 描述。

    @return LaunchDescription 对象。
    """

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    robot_name = LaunchConfiguration("robot_name")
    namespace = LaunchConfiguration("namespace")
    tf_prefix = LaunchConfiguration("tf_prefix")
    map_frame = LaunchConfiguration("map_frame")
    sim_drive_mode = LaunchConfiguration("sim_drive_mode")

    pkg_description = get_package_share_directory("robmini_description")
    pkg_navigation = get_package_share_directory("robmini_navigation")

    # 1. 启动机器人底盘、ros2_control、robot_state_publisher 等。
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

    # 2. 启动 slam_toolbox 建图。
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "bringup_mapping_sim.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
            "robot_name": robot_name,
            "namespace": namespace,
            "tf_prefix": tf_prefix,
            "map_frame": map_frame,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robmini"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("tf_prefix", default_value=""),
            DeclareLaunchArgument("map_frame", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("sim_drive_mode", default_value="planar", choices=["ros2_control", "planar"]),

            # 先启动底盘和 TF。
            simulation_launch,

            # 延迟启动 SLAM，等待底盘、TF、雷达基本就绪。
            TimerAction(
                period=5.0,
                actions=[mapping_launch],
            ),
        ]
    )