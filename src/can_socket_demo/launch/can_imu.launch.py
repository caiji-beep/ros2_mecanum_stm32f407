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


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


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
                "publish_orientation": _as_bool(
                    context.launch_configurations.get("publish_orientation", "false")
                ),
                "gyro_unit": context.launch_configurations.get("gyro_unit", "rad_per_s"),
                "orientation_unit": context.launch_configurations.get("orientation_unit", "rad"),
                "require_calibrated": _as_bool(
                    context.launch_configurations.get("require_calibrated", "false")
                ),
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
        DeclareLaunchArgument("publish_orientation", default_value="false"),
        DeclareLaunchArgument("gyro_unit", default_value="rad_per_s"),
        DeclareLaunchArgument("orientation_unit", default_value="rad"),
        DeclareLaunchArgument("require_calibrated", default_value="false"),
        OpaqueFunction(function=_prepare_node),
    ])
