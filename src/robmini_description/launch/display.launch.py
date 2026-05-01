#!/usr/bin/env python3

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _clean_namespace(value):
    value = str(value or "").strip()
    if value == "/":
        return ""
    return value.strip("/")


def _join_frame(prefix, frame):
    prefix = _clean_namespace(prefix)
    return f"{prefix}/{frame}" if prefix else frame


def _write_rviz_config(template_path, namespace, tf_prefix):
    with open(template_path, "r", encoding="utf-8") as f:
        text = f.read()

    namespace_topic = f"/{namespace}" if namespace else ""
    tf_prefix_text = f"{tf_prefix}/" if tf_prefix else ""
    text = text.replace("/robmini", namespace_topic)
    text = text.replace("robmini/", tf_prefix_text)

    fd, temp_path = tempfile.mkstemp(prefix="robmini_model_rviz_", suffix=".rviz")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(text)
    return temp_path


def _prepare_rviz(context, *args, **kwargs):
    robot_name = context.launch_configurations.get("robot_name", "robmini")
    namespace = _clean_namespace(context.launch_configurations.get("namespace", ""))
    if not namespace:
        namespace = _clean_namespace(robot_name)

    tf_prefix = _clean_namespace(context.launch_configurations.get("tf_prefix", ""))
    if not tf_prefix:
        tf_prefix = namespace

    pkg_share = get_package_share_directory("robmini_description")
    rviz_config = _write_rviz_config(
        os.path.join(pkg_share, "rviz", "model_01.rviz"),
        namespace,
        tf_prefix,
    )

    return [
        SetLaunchConfiguration("rviz_config", rviz_config),
        SetLaunchConfiguration("fixed_frame", _join_frame(tf_prefix, "base_link")),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("robmini_description")

    model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "includes", "model.launch.py")),
        launch_arguments={
            "robot_name": LaunchConfiguration("robot_name"),
            "namespace": LaunchConfiguration("namespace"),
            "tf_prefix": LaunchConfiguration("tf_prefix"),
            "model": LaunchConfiguration("model"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_mock": "true",
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config"), "-f", LaunchConfiguration("fixed_frame")],
        parameters=[{
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robmini"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("model", default_value=os.path.join(pkg_share, "urdf", "robmini_run.urdf.xacro")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_prepare_rviz),
        model_launch,
        rviz_node,
    ])
