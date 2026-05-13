#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


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


def _as_number(value):
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return None
    return None


def _normalise_param_value(value):
    if isinstance(value, list):
        if not value:
            return value

        numbers = [_as_number(item) for item in value]
        if all(number is not None for number in numbers):
            return numbers

        return [_normalise_param_value(item) for item in value]

    if isinstance(value, dict):
        return {key: _normalise_param_value(item) for key, item in value.items()}

    return value


def _load_ekf_params(template_path, namespace, tf_prefix, map_frame, odom_topic, imu_topic):
    with open(template_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    config = _replace_placeholders(config, {
        "${namespace}": namespace,
        "${tf_prefix}": tf_prefix,
        "${base_frame}": _join_frame(tf_prefix, "base_link"),
        "${odom_frame}": _join_frame(tf_prefix, "odom"),
        "${map_frame}": map_frame,
    })

    params = config.setdefault("ekf_filter_node", {}).setdefault("ros__parameters", {})
    params["odom0"] = odom_topic
    params["imu0"] = imu_topic
    return _normalise_param_value(params)


def _prepare_node(context, *args, **kwargs):
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

    use_sim_time = context.launch_configurations.get("use_sim_time", "false")
    odom_topic = context.launch_configurations.get("odom_topic", "odom").strip() or "odom"
    imu_topic = context.launch_configurations.get("imu_topic", "imu/data_raw").strip() or "imu/data_raw"
    filtered_odom_topic = (
        context.launch_configurations.get("filtered_odom_topic", "odometry/filtered").strip()
        or "odometry/filtered"
    )

    pkg_share = get_package_share_directory("robmini_navigation")
    ekf_params = _load_ekf_params(
        os.path.join(pkg_share, "config", "ekf_imu_odom.yaml"),
        namespace,
        tf_prefix,
        map_frame,
        odom_topic,
        imu_topic,
    )

    return [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            namespace=namespace,
            output="screen",
            parameters=[ekf_params, {"use_sim_time": _as_bool(use_sim_time)}],
            remappings=[
                ("odometry/filtered", filtered_odom_topic),
                ("tf", "/tf"),
                ("tf_static", "/tf_static"),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robmini"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("map_frame", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("odom_topic", default_value="odom"),
        DeclareLaunchArgument("imu_topic", default_value="imu/data_raw"),
        DeclareLaunchArgument("filtered_odom_topic", default_value="odometry/filtered"),
        OpaqueFunction(function=_prepare_node),
    ])
