#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction


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

    terminal = context.launch_configurations.get("terminal", "auto").strip().lower()
    cmd_vel_topic = context.launch_configurations.get("cmd_vel_topic", "cmd_vel")
    use_gnome_terminal = terminal == "gnome" or (terminal == "auto" and bool(os.environ.get("DISPLAY")))

    teleop_cmd = [
        "ros2",
        "run",
        "robmini_navigation",
        "mecanum_teleop_keyboard",
        "--ros-args",
        "-p",
        f"cmd_vel_topic:={cmd_vel_topic}",
    ]
    if namespace:
        teleop_cmd.extend(["-r", f"__ns:=/{namespace}"])

    if use_gnome_terminal:
        return [
            ExecuteProcess(
                cmd=["gnome-terminal", "--"] + teleop_cmd,
                output="screen",
            )
        ]

    return [
        LogInfo(msg="No GUI terminal is used. Press teleop keys in this launch terminal."),
        ExecuteProcess(
            cmd=["bash", "-c", 'exec "$@" < /dev/tty', "bash"] + teleop_cmd,
            output="screen",
            emulate_tty=True,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robmini"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("cmd_vel_topic", default_value="cmd_vel"),
        DeclareLaunchArgument(
            "terminal",
            default_value="auto",
            choices=["auto", "gnome", "none"],
            description="Teleop terminal mode. Use 'none' for SSH/headless robots.",
        ),
        OpaqueFunction(function=_prepare_process),
    ])
