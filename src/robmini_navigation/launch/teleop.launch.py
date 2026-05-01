#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction


def _clean_namespace(value):
    value = str(value or "").strip()
    if value == "/":
        return ""
    return value.strip("/")


def _prepare_process(context, *args, **kwargs):
    robot_name = context.launch_configurations.get("robot_name", "robmini")
    namespace = _clean_namespace(context.launch_configurations.get("namespace", ""))
    if not namespace:
        namespace = _clean_namespace(robot_name)

    cmd = [
        "gnome-terminal",
        "--",
        "ros2",
        "run",
        "robmini_navigation",
        "mecanum_teleop_keyboard",
        "--ros-args",
        "-p",
        "cmd_vel_topic:=cmd_vel",
    ]
    if namespace:
        cmd.extend(["-r", f"__ns:=/{namespace}"])

    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robmini"),
        DeclareLaunchArgument("namespace", default_value=""),
        OpaqueFunction(function=_prepare_process),
    ])
