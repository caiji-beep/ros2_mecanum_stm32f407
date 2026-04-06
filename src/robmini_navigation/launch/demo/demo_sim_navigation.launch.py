#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
demo_sim_navigation.launch.py
先启动 simulation.launch.py
→ 5 秒后启动 robot_bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 包路径
    pkg_robot_description = get_package_share_directory('robmini_description')
    pkg_robot_nav2        = get_package_share_directory('robmini_navigation')

    # 1. simulation.launch.py —— 立即启动
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_description, 'launch', 'sim_02_launch.py')
        )
    )

    # 2. robot_bringup.launch.py —— 延迟 5 s 启动
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_nav2, 'launch', 'robot_bringup.launch.py')
        )
    )

    delay_bringup = TimerAction(
        period=12.0,           
        actions=[bringup_launch]
    )

    # 3. 组合并返回
    return LaunchDescription([
        simulation_launch,
        delay_bringup
    ])
