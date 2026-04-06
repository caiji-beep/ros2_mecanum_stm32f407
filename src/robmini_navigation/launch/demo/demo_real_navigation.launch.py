#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
        # ---------------- 声明可调参数 ----------------
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')   # true=仿真, false=真机
    robot_name_arg   = DeclareLaunchArgument('robot_name',   default_value='robmini')

        # -------- 方便后续读取的 LaunchConfiguration --------
    use_sim_time       = LaunchConfiguration('use_sim_time')
    robot_name   = LaunchConfiguration('robot_name')

    # 包路径
    pkg_robot_description = get_package_share_directory('robmini_description')
    pkg_robot_nav2        = get_package_share_directory('robmini_navigation')

    # 1. real_bringup.launch.py —— 立即启动
    real_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_description, 'launch', 'real_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_name': robot_name
        }.items()
    )

    # 2. lidar.launch.py —— 延迟 3 s 启动
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_nav2, 'launch', 'lidar.launch.py')
        ),
        launch_arguments={
            'robot_name': robot_name
        }.items()
    )
    delay_lidar = TimerAction(
        period=3.0,
        actions=[lidar_launch]
    )

    # 2. robot_bringup.launch.py —— 延迟 5 s 启动
    robot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_nav2, 'launch', 'robot_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_name': robot_name
        }.items()
    )

    delay_robot_bringup = TimerAction(
        period=5.0,
        actions=[robot_bringup_launch]
    )

    # 3. 组合并返回
    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        real_bringup_launch,
        delay_lidar,
        delay_robot_bringup
    ])
