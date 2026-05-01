#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file bringup_mapping_real.launch.py
@brief RobMini SLAM 建图启动文件

该 launch 文件用于启动 RobMini 的在线建图功能，主要包括：
1. 根据机器人命名空间和 TF 前缀生成 slam_toolbox 参数文件；
2. 启动 slam_toolbox 异步建图节点；
3. 可选启动 RViz2，用于可视化地图、激光雷达、TF 和建图过程。

适用于实机或仿真环境下的二维激光 SLAM 建图。
"""

import os
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    _join_frame("robmini", "base_link") -> "robmini/base_link"
    _join_frame("", "base_link")        -> "base_link"
    """
    prefix = _clean_namespace(prefix)

    return f"{prefix}/{frame}" if prefix else frame


def _replace_placeholders(obj, replacements):
    """
    @brief 递归替换配置文件中的占位符。

    用于替换 slam_template.yaml 中的 namespace、tf_prefix、
    base_frame、odom_frame 和 map_frame 等占位符。
    """
    if isinstance(obj, str):
        for key, value in replacements.items():
            obj = obj.replace(key, value)
        return obj

    if isinstance(obj, list):
        return [_replace_placeholders(item, replacements) for item in obj]

    if isinstance(obj, dict):
        return {
            key: _replace_placeholders(value, replacements)
            for key, value in obj.items()
        }

    return obj


def _write_rviz_config(template_path, namespace, tf_prefix, map_frame):
    """
    @brief 根据当前机器人参数生成临时 RViz 配置文件。

    RViz 模板中的默认命名空间、TF 前缀和 Fixed Frame 会被替换为
    当前 launch 参数对应的值。
    """

    # 读取 RViz 模板配置。
    with open(template_path, "r", encoding="utf-8") as f:
        text = f.read()

    namespace_topic = f"/{namespace}" if namespace else ""
    tf_prefix_text = f"{tf_prefix}/" if tf_prefix else ""
    default_map_frame = _join_frame(tf_prefix, "map")

    # 替换模板中的默认话题命名空间和 TF 前缀。
    text = text.replace("/robmini", namespace_topic)
    text = text.replace("robmini/", tf_prefix_text)

    # 当用户指定 map_frame 时，同步替换 RViz 的 Fixed Frame。
    if default_map_frame != map_frame:
        text = text.replace(
            f"Fixed Frame: {default_map_frame}",
            f"Fixed Frame: {map_frame}",
        )
        text = text.replace(
            f"        {default_map_frame}:",
            f"        {map_frame}:",
        )
        text = text.replace(
            f"      {default_map_frame}:",
            f"      {map_frame}:",
        )

    # 写入临时 RViz 配置文件。
    fd, temp_path = tempfile.mkstemp(
        prefix="robmini_mapping_rviz_",
        suffix=".rviz",
    )

    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(text)

    return temp_path


def _gen_temp_yaml(context, *args, **kwargs):
    """
    @brief 生成 SLAM 参数文件和 RViz 配置文件。

    该函数在 launch 运行阶段读取参数，解析 namespace、tf_prefix、
    map_frame，并基于模板文件生成临时配置。
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

    # 读取地图坐标系；若未指定，则默认使用 <tf_prefix>/map。
    map_frame = context.launch_configurations.get("map_frame", "").strip()

    if not map_frame:
        map_frame = _join_frame(tf_prefix, "map")

    # 获取 robmini_navigation 功能包路径。
    pkg_share = FindPackageShare("robmini_navigation").perform(context)
    template_path = os.path.join(pkg_share, "config", "slam_template.yaml")

    # 读取 slam_toolbox 参数模板。
    with open(template_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    # 替换 SLAM 参数模板中的占位符。
    data = _replace_placeholders(
        data,
        {
            "${namespace}": namespace,
            "${tf_prefix}": tf_prefix,
            "${base_frame}": _join_frame(tf_prefix, "base_link"),
            "${odom_frame}": _join_frame(tf_prefix, "odom"),
            "${map_frame}": map_frame,
        },
    )

    # 写入临时 SLAM 参数文件。
    fd, tmp_path = tempfile.mkstemp(
        prefix="robmini_slam_",
        suffix=".yaml",
    )

    with os.fdopen(fd, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)

    # 生成与当前命名空间和 TF 前缀匹配的 RViz 配置文件。
    rviz_config = _write_rviz_config(
        os.path.join(pkg_share, "rviz", "bringup_mapping.rviz"),
        namespace,
        tf_prefix,
        map_frame,
    )

    return [
        # 将运行时生成的配置结果写入 launch 配置，供后续节点使用。
        SetLaunchConfiguration("slam_yaml", tmp_path),
        SetLaunchConfiguration("resolved_namespace", namespace),
        SetLaunchConfiguration("rviz_config", rviz_config),
        SetLaunchConfiguration("resolved_map_frame", map_frame),
    ]


def generate_launch_description():
    """
    @brief 生成 SLAM 建图系统的 launch 描述。

    @return LaunchDescription 对象。
    """

    # 运行时生成或解析后的 launch 配置。
    slam_yaml = LaunchConfiguration("slam_yaml")
    namespace = LaunchConfiguration("resolved_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    map_frame = LaunchConfiguration("resolved_map_frame")

    # slam_toolbox 异步建图节点。
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=namespace,
        parameters=[
            slam_yaml,
            {
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=[
            # 将全局地图话题重映射到当前命名空间下。
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/map_updates", "map_updates"),
        ],
        output="screen",
    )

    # 可选启动 RViz2，用于查看建图状态。
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", rviz_config, "-f", map_frame],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

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

            # 地图坐标系名称。
            # 若为空，则默认生成 <tf_prefix>/map。
            DeclareLaunchArgument(
                "map_frame",
                default_value="",
            ),

            # 是否使用仿真时间。
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),

            # 是否启动 RViz2。
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
            ),

            # 运行时生成 SLAM 和 RViz 临时配置。
            OpaqueFunction(function=_gen_temp_yaml),

            # 启动 SLAM 建图节点。
            slam_node,

            # 按需启动 RViz2 可视化界面。
            rviz_node,
        ]
    )