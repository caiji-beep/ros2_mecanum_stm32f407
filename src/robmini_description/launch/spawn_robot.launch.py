#!/usr/bin/env python3

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


def _as_float(value, default):
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return default


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
    sim_drive_mode = context.launch_configurations.get("sim_drive_mode", "planar").strip()
    if sim_drive_mode not in ("ros2_control", "planar"):
        raise ValueError("sim_drive_mode must be 'ros2_control' or 'planar'")

    spawn_delay = _as_float(context.launch_configurations.get("spawn_delay", "0.0"), 0.0)
    initial_pose_x = context.launch_configurations.get("initial_pose_x", "0.0")
    initial_pose_y = context.launch_configurations.get("initial_pose_y", "0.0")
    initial_pose_z = context.launch_configurations.get("initial_pose_z", "0.078")
    initial_pose_a = context.launch_configurations.get("initial_pose_a", "0.0")

    controller_manager_path = f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    pkg_share = get_package_share_directory("robmini_description")
    xacro_file = os.path.join(pkg_share, "urdf", "robmini_run.urdf.xacro")

    controller_yaml = ""
    if sim_drive_mode == "ros2_control":
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
            " sim_drive_mode:=",
            sim_drive_mode,
            " controller_config:=",
            controller_yaml,
        ]),
        value_type=str,
    )

    robot_description_topic = f"/{namespace}/robot_description" if namespace else "/robot_description"

    actions = [
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
            remappings=[
                ("tf", "/tf"),
                ("tf_static", "/tf_static"),
                ("robot_description", robot_description_topic),
                ("/robot_description", robot_description_topic),
            ],
        ),
        TimerAction(
            period=spawn_delay,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    output="screen",
                    arguments=[
                        "-topic",
                        robot_description_topic,
                        "-entity",
                        namespace if namespace else robot_name,
                        "-robot_namespace",
                        namespace,
                        "-x",
                        initial_pose_x,
                        "-y",
                        initial_pose_y,
                        "-z",
                        initial_pose_z,
                        "-Y",
                        initial_pose_a,
                    ],
                )
            ],
        ),
    ]

    if sim_drive_mode == "ros2_control":
        actions.extend([
            TimerAction(
                period=spawn_delay + 5.0,
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
                period=spawn_delay + 7.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["mecanum_drive_controller", "--controller-manager", controller_manager_path],
                        output="screen",
                    )
                ],
            ),
        ])
    else:
        actions.append(
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                namespace=namespace,
                name="joint_state_publisher",
                parameters=[{
                    "robot_description": robot_description,
                    "rate": 30,
                    "use_sim_time": use_sim_time_bool,
                }],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robmini"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("sim_drive_mode", default_value="planar", choices=["ros2_control", "planar"]),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_z", default_value="0.078"),
        DeclareLaunchArgument("initial_pose_a", default_value="0.0"),
        DeclareLaunchArgument("spawn_delay", default_value="0.0"),
        OpaqueFunction(function=_prepare_robot),
    ])
