#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file demo_real_navigation.launch.py
@brief RobMini 实机导航总启动文件

该 launch 文件用于按顺序启动 RobMini 实机运行所需的核心模块：
1. real_bringup.launch.py：机器人本体、ros2_control、状态发布等；
2. lidar.launch.py：激光雷达驱动；
3. robot_bringup.launch.py：Nav2 地图、定位与导航模块。

启动顺序通过 TimerAction 控制，避免导航节点在底层控制或雷达尚未就绪时提前启动。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    @brief 生成 RobMini 实机导航系统的 launch 描述。

    @return LaunchDescription 对象。
    """

    # 公共 launch 参数，用于传递给各个子 launch 文件。
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    robot_name = LaunchConfiguration("robot_name")
    namespace = LaunchConfiguration("namespace")
    tf_prefix = LaunchConfiguration("tf_prefix")
    map_frame = LaunchConfiguration("map_frame")
    map_file = LaunchConfiguration("map_file")
    use_ekf = LaunchConfiguration("use_ekf")
    can_interface = LaunchConfiguration("can_interface")
    imu_topic = LaunchConfiguration("imu_topic")
    filtered_odom_topic = LaunchConfiguration("filtered_odom_topic")
    publish_imu_orientation = LaunchConfiguration("publish_imu_orientation")
    imu_gyro_unit = LaunchConfiguration("imu_gyro_unit")
    imu_orientation_unit = LaunchConfiguration("imu_orientation_unit")

    # 获取功能包 share 路径。
    pkg_can = get_package_share_directory("can_socket_demo")
    pkg_description = get_package_share_directory("robmini_description")
    pkg_navigation = get_package_share_directory("robmini_navigation")

    # 启动机器人本体相关节点：
    # 包括 robot_state_publisher、ros2_control、控制器等。
    real_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, "launch", "real_bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_name": robot_name,
            "namespace": namespace,
            "tf_prefix": tf_prefix,
            "use_ekf": use_ekf,
        }.items(),
    )

    # 启动激光雷达驱动。
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "lidar.launch.py")
        ),
        launch_arguments={
            "robot_name": robot_name,
            "namespace": namespace,
            "tf_prefix": tf_prefix,
        }.items(),
    )

    can_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_can, "launch", "can_imu.launch.py")
        ),
        condition=IfCondition(use_ekf),
        launch_arguments={
            "robot_name": robot_name,
            "namespace": namespace,
            "tf_prefix": tf_prefix,
            "interface": can_interface,
            "imu_topic": imu_topic,
            "publish_orientation": publish_imu_orientation,
            "gyro_unit": imu_gyro_unit,
            "orientation_unit": imu_orientation_unit,
        }.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "ekf_odom.launch.py")
        ),
        condition=IfCondition(use_ekf),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_name": robot_name,
            "namespace": namespace,
            "tf_prefix": tf_prefix,
            "map_frame": map_frame,
            "odom_topic": "odom",
            "imu_topic": imu_topic,
            "filtered_odom_topic": filtered_odom_topic,
        }.items(),
    )

    # 启动 Nav2 导航模块：
    # 包括地图服务器、AMCL 定位、路径规划、控制器等。
    nav_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "robot_bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_name": robot_name,
            "namespace": namespace,
            "tf_prefix": tf_prefix,
            "map_frame": map_frame,
            "use_rviz": use_rviz,
            "map_file": map_file,
            "use_ekf": use_ekf,
            "filtered_odom_topic": filtered_odom_topic,
        }.items(),
    )

    return LaunchDescription(
        [
            # 是否使用仿真时间。
            # 实机运行通常为 false。
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),

            # 是否启动 RViz。
            # 默认 false，可通过命令行 use_rviz:=true 开启。
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
            ),

            # 机器人名称。
            DeclareLaunchArgument(
                "robot_name",
                default_value="robmini",
            ),

            # ROS 命名空间。
            # 多机器人场景下用于隔离节点、话题和服务。
            DeclareLaunchArgument(
                "namespace",
                default_value="",
            ),

            # TF 前缀。
            # 多机器人场景下用于区分 base_link、odom 等坐标系。
            DeclareLaunchArgument(
                "tf_prefix",
                default_value="",
            ),

            # 地图坐标系名称。
            # 若为空，通常由子 launch 文件按默认规则生成。
            DeclareLaunchArgument(
                "map_frame",
                default_value="",
            ),

            DeclareLaunchArgument(
                "map_file",
                default_value="room_mini/115_map.yaml",
            ),
            DeclareLaunchArgument(
                "use_ekf",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="imu/data_raw",
            ),
            DeclareLaunchArgument(
                "filtered_odom_topic",
                default_value="odometry/filtered",
            ),
            DeclareLaunchArgument(
                "publish_imu_orientation",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "imu_gyro_unit",
                default_value="rad_per_s",
            ),
            DeclareLaunchArgument(
                "imu_orientation_unit",
                default_value="rad",
            ),

            # 先启动机器人底层。
            real_bringup_launch,

            # 延时启动雷达，等待底层节点初始化完成。
            TimerAction(
                period=3.0,
                actions=[lidar_launch],
            ),
            TimerAction(
                period=3.0,
                actions=[can_imu_launch],
            ),
            TimerAction(
                period=4.0,
                actions=[ekf_launch],
            ),

            # 延时启动导航，等待机器人状态和雷达数据基本就绪。
            TimerAction(
                period=5.0,
                actions=[nav_bringup_launch],
            ),
        ]
    )
