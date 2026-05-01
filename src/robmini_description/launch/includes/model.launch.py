#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _clean_namespace(value):
    value = str(value or "").strip()
    if value == "/":
        return ""
    return value.strip("/")


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _prepare_nodes(context, *args, **kwargs):
    robot_name = context.launch_configurations.get("robot_name", "robmini")
    namespace = _clean_namespace(context.launch_configurations.get("namespace", ""))
    if not namespace:
        namespace = _clean_namespace(robot_name)

    tf_prefix = _clean_namespace(context.launch_configurations.get("tf_prefix", ""))
    if not tf_prefix:
        tf_prefix = namespace

    model = LaunchConfiguration("model").perform(context)
    use_sim_time = context.launch_configurations.get("use_sim_time", "false")
    use_sim_time_bool = _as_bool(use_sim_time)
    use_mock = context.launch_configurations.get("use_mock", "true")
    robot_description_topic = f"/{namespace}/robot_description" if namespace else "/robot_description"

    robot_description = ParameterValue(
        Command([
            "xacro ",
            model,
            " prefix:=",
            tf_prefix,
            " namespace:=",
            namespace,
            " use_mock:=",
            use_mock,
        ]),
        value_type=str,
    )

    tf_remappings = [("tf", "/tf"), ("tf_static", "/tf_static")]
    description_remappings = [
        ("robot_description", robot_description_topic),
        ("/robot_description", robot_description_topic),
    ]

    return [
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            namespace=namespace,
            parameters=[{
                "robot_description": robot_description,
                "rate": 50,
                "use_sim_time": use_sim_time_bool,
            }],
            remappings=description_remappings,
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=namespace,
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "publish_frequency": 50.0,
                "use_sim_time": use_sim_time_bool,
            }],
            remappings=tf_remappings + description_remappings,
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("robmini_description")

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robmini"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument(
            "model",
            default_value=os.path.join(pkg_share, "urdf", "robmini_run.urdf.xacro"),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_mock", default_value="true"),
        OpaqueFunction(function=_prepare_nodes),
    ])
