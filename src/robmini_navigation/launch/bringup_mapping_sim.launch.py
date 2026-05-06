#!/usr/bin/env python3

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
    value = str(value or "").strip()
    if value == "/":
        return ""
    return value.strip("/")


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


def _write_rviz_config(template_path, namespace, tf_prefix, map_frame):
    with open(template_path, "r", encoding="utf-8") as f:
        text = f.read()

    namespace_topic = f"/{namespace}" if namespace else ""
    tf_prefix_text = f"{tf_prefix}/" if tf_prefix else ""
    default_map_frame = _join_frame(tf_prefix, "map")

    text = text.replace("/robmini", namespace_topic)
    text = text.replace("robmini/", tf_prefix_text)
    if default_map_frame != map_frame:
        text = text.replace(f"Fixed Frame: {default_map_frame}", f"Fixed Frame: {map_frame}")
        text = text.replace(f"        {default_map_frame}:", f"        {map_frame}:")
        text = text.replace(f"      {default_map_frame}:", f"      {map_frame}:")

    fd, temp_path = tempfile.mkstemp(prefix="robmini_mapping_rviz_", suffix=".rviz")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(text)
    return temp_path


def _gen_temp_yaml(context, *args, **kwargs):
    robot_name = context.launch_configurations.get("robot_name", "robmini")
    namespace = _clean_namespace(context.launch_configurations.get("namespace", ""))
    if not namespace:
        namespace = _clean_namespace(robot_name)

    tf_prefix = _clean_namespace(context.launch_configurations.get("tf_prefix", ""))
    if not tf_prefix:
        tf_prefix = namespace

    map_frame = context.launch_configurations.get("map_frame", "").strip()
    if not map_frame:
        map_frame = _join_frame(tf_prefix, "map")
    scan_topic = context.launch_configurations.get("scan_topic", "scan").strip() or "scan"

    pkg_share = FindPackageShare("robmini_navigation").perform(context)
    template_path = os.path.join(pkg_share, "config", "slam_template.yaml")

    with open(template_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    data = _replace_placeholders(data, {
        "${namespace}": namespace,
        "${tf_prefix}": tf_prefix,
        "${base_frame}": _join_frame(tf_prefix, "base_link"),
        "${odom_frame}": _join_frame(tf_prefix, "odom"),
        "${map_frame}": map_frame,
        "${scan_topic}": scan_topic,
    })

    fd, tmp_path = tempfile.mkstemp(prefix="robmini_slam_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)

    rviz_config = _write_rviz_config(
        os.path.join(pkg_share, "rviz", "bringup_mapping.rviz"),
        namespace,
        tf_prefix,
        map_frame,
    )

    return [
        SetLaunchConfiguration("slam_yaml", tmp_path),
        SetLaunchConfiguration("resolved_namespace", namespace),
        SetLaunchConfiguration("rviz_config", rviz_config),
        SetLaunchConfiguration("resolved_map_frame", map_frame),
    ]


def generate_launch_description():
    namespace = LaunchConfiguration("resolved_namespace")
    slam_yaml = LaunchConfiguration("slam_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    map_frame = LaunchConfiguration("resolved_map_frame")

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robmini"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("map_frame", default_value=""),
        DeclareLaunchArgument("scan_topic", default_value="scan"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=_gen_temp_yaml),
        Node(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox",
            namespace=namespace,
            parameters=[slam_yaml, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/map", "map"),
                ("/map_metadata", "map_metadata"),
                ("/map_updates", "map_updates"),
            ],
            output="screen",
        ),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz",
            output="screen",
            arguments=["-d", rviz_config, "-f", map_frame],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
