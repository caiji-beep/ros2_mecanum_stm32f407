#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file real_bringup.launch.py
@brief RobMini 机器人 ros2_control 与 robot_state_publisher 启动文件

该 launch 文件用于启动 RobMini 机器人的运行时控制相关节点，主要包括：
1. ros2_control_node：加载机器人描述与控制器配置；
2. robot_state_publisher：发布机器人 TF 变换；
3. joint_state_broadcaster：发布关节状态；
4. mecanum_drive_controller：启动麦克纳姆轮底盘控制器。

支持通过 launch 参数配置：
- robot_name：机器人名称；
- namespace：ROS 命名空间；
- tf_prefix：TF 前缀，用于多机器人场景；
- use_sim_time：是否使用仿真时间；
- use_mock：是否使用 mock 硬件接口。

典型用途：
- 单机器人实机运行；
- 多机器人命名空间隔离；
- mock 硬件调试；
- Gazebo / Isaac Sim 等仿真环境中的控制测试。
"""


import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _clean_namespace(value):
    value = str(value or "").strip()
    if value == "/":
        return ""
    return value.strip("/")


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _join_frame(prefix, frame):
    prefix = _clean_namespace(prefix)
    return f"{prefix}/{frame}" if prefix else frame


def _replace_placeholders(obj, replacements):
    if isinstance(obj, str):
        for key, value in replacements.items():
            obj = obj.replace(key, value)
        return obj
    if isinstance(obj, list):
        return [_replace_placeholders(item, replacements) for item in obj]
    if isinstance(obj, dict):
        return {key: _replace_placeholders(value, replacements) for key, value in obj.items()}
    return obj


def _write_controller_yaml(controller_yaml, tf_prefix):
    with open(controller_yaml, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    config = _replace_placeholders(config, {
        "${tf_prefix}": _clean_namespace(tf_prefix),
        "${base_frame}": _join_frame(tf_prefix, "base_link"),
        "${odom_frame}": _join_frame(tf_prefix, "odom"),
    })

    fd, temp_path = tempfile.mkstemp(prefix="robmini_controllers_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, sort_keys=False)
    return temp_path


def _prepare_nodes(context, *args, **kwargs):
    robot_name = context.launch_configurations.get("robot_name", "robmini")
    namespace = _clean_namespace(context.launch_configurations.get("namespace", ""))
    if not namespace:
        namespace = _clean_namespace(robot_name)

    tf_prefix = _clean_namespace(context.launch_configurations.get("tf_prefix", ""))
    if not tf_prefix:
        tf_prefix = namespace

    use_mock = context.launch_configurations.get("use_mock", "false")
    use_sim_time = context.launch_configurations.get("use_sim_time", "false")
    use_sim_time_bool = _as_bool(use_sim_time)

    pkg_share = get_package_share_directory("robmini_description")
    xacro_file = os.path.join(pkg_share, "urdf", "robmini_run.urdf.xacro")
    controller_yaml = _write_controller_yaml(
        os.path.join(pkg_share, "config", "robmini_mecanum_controllers.yaml"),
        tf_prefix,
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_file,
            " prefix:=",
            tf_prefix,
            " namespace:=",
            namespace,
            " use_mock:=",
            use_mock,
            " use_gazebo:=false",
            " controller_config:=",
            controller_yaml,
        ]),
        value_type=str,
    )

    controller_manager_path = f"/{namespace}/controller_manager" if namespace else "/controller_manager"

    return [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=namespace,
            output="screen",
            parameters=[
                {"robot_description": robot_description, "use_sim_time": use_sim_time_bool},
                controller_yaml,
            ],
            remappings=[
                ("mecanum_drive_controller/reference_unstamped", "cmd_vel"),
                ("mecanum_drive_controller/odometry", "odom"),
                ("mecanum_drive_controller/tf_odometry", "/tf"),
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "publish_frequency": 50.0,
                "use_sim_time": use_sim_time_bool,
            }],
            remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster",
                        "--controller-manager",
                        controller_manager_path,
                    ],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "mecanum_drive_controller",
                        "--controller-manager",
                        controller_manager_path,
                    ],
                    output="screen",
                )
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robmini"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_mock", default_value="false"),
        OpaqueFunction(function=_prepare_nodes),
    ])
