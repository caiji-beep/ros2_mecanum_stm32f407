#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file demo_real_mapping.launch.py
@brief RobMini 实机建图总启动文件

该 launch 文件用于按顺序启动 RobMini 实机建图所需模块：
1. real_bringup.launch.py：机器人本体、ros2_control、状态发布等；
2. lidar.launch.py：激光雷达驱动；
3. bringup_mapping_real.launch.py：slam_toolbox 建图和可选 RViz。

启动顺序通过 TimerAction 控制，避免 SLAM 在底盘、TF 或雷达尚未就绪时提前启动。
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
    @brief 生成 RobMini 实机建图系统的 launch 描述。

    @return LaunchDescription 对象。
    """

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    robot_name = LaunchConfiguration("robot_name")
    namespace = LaunchConfiguration("namespace")
    tf_prefix = LaunchConfiguration("tf_prefix")
    map_frame = LaunchConfiguration("map_frame")
    use_ekf = LaunchConfiguration("use_ekf")
    can_interface = LaunchConfiguration("can_interface")
    imu_topic = LaunchConfiguration("imu_topic")
    filtered_odom_topic = LaunchConfiguration("filtered_odom_topic")
    publish_imu_orientation = LaunchConfiguration("publish_imu_orientation")
    imu_gyro_unit = LaunchConfiguration("imu_gyro_unit")
    imu_orientation_unit = LaunchConfiguration("imu_orientation_unit")

    pkg_can = get_package_share_directory("can_socket_demo")
    pkg_description = get_package_share_directory("robmini_description")
    pkg_navigation = get_package_share_directory("robmini_navigation")

    # 1. 启动机器人底盘、ros2_control、robot_state_publisher 等。
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

    # 2. 启动激光雷达。
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

    # 3. 启动 slam_toolbox 建图。
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "bringup_mapping_real.launch.py")
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
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),

            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
            ),

            DeclareLaunchArgument(
                "robot_name",
                default_value="robmini",
            ),

            DeclareLaunchArgument(
                "namespace",
                default_value="",
            ),

            DeclareLaunchArgument(
                "tf_prefix",
                default_value="",
            ),

            DeclareLaunchArgument(
                "map_frame",
                default_value="",
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

            # 先启动底盘和 TF。
            real_bringup_launch,

            # 延迟启动雷达。
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

            # 延迟启动 SLAM，等待底盘、TF、雷达基本就绪。
            TimerAction(
                period=5.0,
                actions=[mapping_launch],
            ),
        ]
    )
