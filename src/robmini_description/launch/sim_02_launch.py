#!/usr/bin/env python3

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    fd, temp_path = tempfile.mkstemp(prefix="robmini_gazebo_controllers_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, sort_keys=False)
    return temp_path


def _prepare_robot(context, *args, **kwargs):
    robot_name = context.launch_configurations.get("robot_name", "robmini")
    namespace = _clean_namespace(context.launch_configurations.get("namespace", ""))
    if not namespace:
        namespace = _clean_namespace(robot_name)

    tf_prefix = _clean_namespace(context.launch_configurations.get("tf_prefix", ""))
    if not tf_prefix:
        tf_prefix = namespace

    use_sim_time = context.launch_configurations.get("use_sim_time", "true")
    use_sim_time_bool = _as_bool(use_sim_time)
    controller_manager_path = f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    pkg_share = get_package_share_directory("robmini_description")

    xacro_file = os.path.join(
        pkg_share,
        "urdf",
        "robmini_run.urdf.xacro",
    )
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
            " use_gazebo:=true",
            " use_mock:=false",
            " controller_config:=",
            controller_yaml,
        ]),
        value_type=str,
    )

    return [
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
            period=5.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    output="screen",
                    arguments=[
                        "-topic",
                        f"/{namespace}/robot_description" if namespace else "/robot_description",
                        "-entity",
                        namespace if namespace else robot_name,
                        "-robot_namespace",
                        namespace,
                        "-x",
                        "0",
                        "-y",
                        "0",
                        "-z",
                        "0.078",
                    ],
                )
            ],
        ),
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster", "--controller-manager", controller_manager_path],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["mecanum_drive_controller", "--controller-manager", controller_manager_path],
                    output="screen",
                )
            ],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("robmini_description")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "paused": LaunchConfiguration("paused"),
            "headless": LaunchConfiguration("headless"),
            "debug": LaunchConfiguration("debug"),
            "verbose": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robmini"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=os.path.join(pkg_share, "worlds", "room_mini.world")),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("debug", default_value="false"),
        gazebo_launch,
        OpaqueFunction(function=_prepare_robot),
    ])
