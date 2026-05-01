#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file lidar.launch.py
@brief RobMini 激光雷达启动文件

该 launch 文件用于启动 RPLIDAR 激光雷达节点，并根据机器人命名空间和
TF 前缀设置激光雷达坐标系 frame_id。

主要功能：
1. 启动 rplidar_ros 驱动节点；
2. 支持串口、波特率、扫描模式等参数配置；
3. 支持多机器人场景下的 namespace 和 tf_prefix。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _clean_namespace(value):
    """
    @brief 规范化命名空间字符串。

    去除首尾空格和斜杠，避免生成异常 namespace 或 TF 前缀。
    """
    value = str(value or "").strip()

    if value == "/":
        return ""

    return value.strip("/")


def _join_frame(prefix, frame):
    """
    @brief 拼接 TF 前缀和坐标系名称。

    @example
    _join_frame("robmini", "laser") -> "robmini/laser"
    _join_frame("", "laser")        -> "laser"
    """
    prefix = _clean_namespace(prefix)

    return f"{prefix}/{frame}" if prefix else frame


def _as_bool(value):
    """
    @brief 将 launch 字符串参数转换为布尔值。
    """
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _prepare_node(context, *args, **kwargs):
    """
    @brief 根据 launch 参数动态创建 RPLIDAR 节点。

    使用 OpaqueFunction 的原因是部分参数需要在运行阶段读取，
    并参与 namespace、tf_prefix 和 laser_frame 的计算。
    """

    # 读取机器人名称，默认值为 robmini。
    robot_name = context.launch_configurations.get("robot_name", "robmini")

    # 读取命名空间；若未指定，则默认使用 robot_name。
    namespace = _clean_namespace(
        context.launch_configurations.get("namespace", "")
    )

    if not namespace:
        namespace = _clean_namespace(robot_name)

    # 读取 TF 前缀；若未指定，则默认使用 namespace。
    tf_prefix = _clean_namespace(
        context.launch_configurations.get("tf_prefix", "")
    )

    if not tf_prefix:
        tf_prefix = namespace

    # 读取激光雷达坐标系名称；若未指定，则默认使用 <tf_prefix>/laser。
    laser_frame = context.launch_configurations.get("laser_frame", "").strip()

    if not laser_frame:
        laser_frame = _join_frame(tf_prefix, "laser")

    return [
        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            namespace=namespace,
            output="screen",
            parameters=[
                {
                    # 通信方式，常用值为 serial。
                    "channel_type": context.launch_configurations.get(
                        "channel_type",
                        "serial",
                    ),

                    # 雷达串口设备名。
                    "serial_port": context.launch_configurations.get(
                        "serial_port",
                        "/dev/ttyRadar",
                    ),

                    # 串口波特率。
                    "serial_baudrate": int(
                        context.launch_configurations.get(
                            "serial_baudrate",
                            "115200",
                        )
                    ),

                    # 激光雷达 TF 坐标系名称。
                    "frame_id": laser_frame,

                    # 是否反转扫描方向。
                    "inverted": _as_bool(
                        context.launch_configurations.get(
                            "inverted",
                            "false",
                        )
                    ),

                    # 是否进行角度补偿。
                    "angle_compensate": _as_bool(
                        context.launch_configurations.get(
                            "angle_compensate",
                            "true",
                        )
                    ),

                    # 雷达扫描模式。
                    "scan_mode": context.launch_configurations.get(
                        "scan_mode",
                        "Standard",
                    ),
                }
            ],
        )
    ]


def generate_launch_description():
    """
    @brief 生成激光雷达 launch 描述对象。

    @return LaunchDescription 对象。
    """

    return LaunchDescription(
        [
            # 机器人名称。
            DeclareLaunchArgument(
                "robot_name",
                default_value="robmini",
            ),

            # ROS 命名空间。
            DeclareLaunchArgument(
                "namespace",
                default_value="",
            ),

            # TF 前缀。
            DeclareLaunchArgument(
                "tf_prefix",
                default_value="",
            ),

            # 激光雷达坐标系名称。
            # 若为空，则默认生成 <tf_prefix>/laser。
            DeclareLaunchArgument(
                "laser_frame",
                default_value="",
            ),

            # 雷达通信方式。
            DeclareLaunchArgument(
                "channel_type",
                default_value="serial",
            ),

            # 雷达串口设备路径。
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyRadar",
            ),

            # 雷达串口波特率。
            DeclareLaunchArgument(
                "serial_baudrate",
                default_value="115200",
            ),

            # 是否反转雷达扫描方向。
            DeclareLaunchArgument(
                "inverted",
                default_value="false",
            ),

            # 是否启用角度补偿。
            DeclareLaunchArgument(
                "angle_compensate",
                default_value="true",
            ),

            # 雷达扫描模式。
            DeclareLaunchArgument(
                "scan_mode",
                default_value="Standard",
            ),

            # 运行时读取参数并创建节点。
            OpaqueFunction(function=_prepare_node),
        ]
    )