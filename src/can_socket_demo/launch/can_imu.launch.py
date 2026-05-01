#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _clean_namespace(value):
    value = str(value or "").strip()
    if value == "/":
        return ""
    return value.strip("/")


def _join_frame(prefix, frame):
    prefix = _clean_namespace(prefix)
    return f"{prefix}/{frame}" if prefix else frame


def _prepare_node(context, *args, **kwargs):
    robot_name = context.launch_configurations.get("robot_name", "robmini")
    namespace = _clean_namespace(context.launch_configurations.get("namespace", ""))
    if not namespace:
        namespace = _clean_namespace(robot_name)

    tf_prefix = _clean_namespace(context.launch_configurations.get("tf_prefix", ""))
    if not tf_prefix:
        tf_prefix = namespace

    frame_id = context.launch_configurations.get("frame_id", "").strip()
    if not frame_id:
        frame_id = _join_frame(tf_prefix, "imu_link")

    return [
        Node(
            package="can_socket_demo",
            executable="can_imu_node",
            name="can_imu_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "interface": context.launch_configurations.get("interface", "can0"),
                "imu_topic": context.launch_configurations.get("imu_topic", "imu/data_raw"),
                "frame_id": frame_id,
                "tf_prefix": tf_prefix,
            }],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robmini"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("interface", default_value="can0"),
        DeclareLaunchArgument("imu_topic", default_value="imu/data_raw"),
        DeclareLaunchArgument("frame_id", default_value=""),
        OpaqueFunction(function=_prepare_node),
    ])
