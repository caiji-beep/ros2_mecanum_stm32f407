#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _clean_namespace(value):
    value = str(value or "").strip()
    if value == "/":
        return ""
    return value.strip("/")


def _join_frame(prefix, frame):
    prefix = _clean_namespace(prefix)
    return f"{prefix}/{frame}" if prefix else frame


def _as_float(value, default):
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return default


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _robot_settings(context, index):
    robot_name = context.launch_configurations.get(f"robot_name_{index}", f"robmini_0{index}")
    namespace = _clean_namespace(context.launch_configurations.get(f"namespace_{index}", ""))
    if not namespace:
        namespace = _clean_namespace(robot_name)

    tf_prefix = _clean_namespace(context.launch_configurations.get(f"tf_prefix_{index}", ""))
    if not tf_prefix:
        tf_prefix = namespace

    return {
        "robot_name": robot_name,
        "namespace": namespace,
        "tf_prefix": tf_prefix,
        "map_frame": _join_frame(tf_prefix, "map"),
        "initial_pose_x": context.launch_configurations.get(f"initial_pose_x_{index}", "0.0"),
        "initial_pose_y": context.launch_configurations.get(f"initial_pose_y_{index}", "0.0"),
        "initial_pose_z": context.launch_configurations.get(f"initial_pose_z_{index}", "0.078"),
        "initial_pose_a": context.launch_configurations.get(f"initial_pose_a_{index}", "0.0"),
    }


def _prepare_multi_robot_nodes(context, *args, **kwargs):
    pkg_description = get_package_share_directory("robmini_description")
    pkg_navigation = get_package_share_directory("robmini_navigation")

    robot_1 = _robot_settings(context, 1)
    robot_2 = _robot_settings(context, 2)

    use_sim_time = context.launch_configurations.get("use_sim_time", "true")
    sim_drive_mode = context.launch_configurations.get("sim_drive_mode", "planar")
    map_file = context.launch_configurations.get("map_file", "room_mini/room_mini.yaml")
    use_rviz = context.launch_configurations.get("use_rviz", "true")

    spawn_delay = _as_float(context.launch_configurations.get("delay_spawn", "4.0"), 4.0)
    bringup_delay = _as_float(context.launch_configurations.get("delay_bringup", "10.0"), 10.0)
    rviz_delay = _as_float(context.launch_configurations.get("delay_rviz", "14.0"), 14.0)

    spawn_launch = os.path.join(pkg_description, "launch", "spawn_robot.launch.py")
    bringup_launch = os.path.join(pkg_navigation, "launch", "robot_bringup.launch.py")

    def spawn_robot(robot):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={
                "robot_name": robot["robot_name"],
                "namespace": robot["namespace"],
                "tf_prefix": robot["tf_prefix"],
                "use_sim_time": use_sim_time,
                "sim_drive_mode": sim_drive_mode,
                "initial_pose_x": robot["initial_pose_x"],
                "initial_pose_y": robot["initial_pose_y"],
                "initial_pose_z": robot["initial_pose_z"],
                "initial_pose_a": robot["initial_pose_a"],
                "spawn_delay": "1.0",
            }.items(),
        )

    def bringup_robot(robot):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                "robot_name": robot["robot_name"],
                "namespace": robot["namespace"],
                "tf_prefix": robot["tf_prefix"],
                "map_frame": robot["map_frame"],
                "use_sim_time": use_sim_time,
                "use_rviz": "false",
                "map_file": map_file,
                "initial_pose_x": robot["initial_pose_x"],
                "initial_pose_y": robot["initial_pose_y"],
                "initial_pose_a": robot["initial_pose_a"],
                "use_ekf": "false",
            }.items(),
        )

    def world_to_map_tf(robot):
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"world_to_{robot['tf_prefix'].replace('/', '_')}_map",
            arguments=["0", "0", "0", "0", "0", "0", "world", robot["map_frame"]],
        )

    rviz_config = os.path.join(pkg_navigation, "rviz", "multi_robots_navigation.rviz")

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz_multi_robot_navigation",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": _as_bool(use_sim_time)}],
    )

    return [
        TimerAction(period=spawn_delay, actions=[spawn_robot(robot_1)]),
        TimerAction(period=spawn_delay + 1.0, actions=[spawn_robot(robot_2)]),
        TimerAction(period=bringup_delay, actions=[bringup_robot(robot_1)]),
        TimerAction(period=bringup_delay + 1.0, actions=[bringup_robot(robot_2)]),
        TimerAction(period=bringup_delay + 2.0, actions=[world_to_map_tf(robot_1)]),
        TimerAction(period=bringup_delay + 2.5, actions=[world_to_map_tf(robot_2)]),
        TimerAction(period=rviz_delay, actions=[rviz_node]),
    ]


def generate_launch_description():
    pkg_description = get_package_share_directory("robmini_description")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world_name"),
            "gui": LaunchConfiguration("gui"),
            "paused": LaunchConfiguration("paused"),
            "headless": LaunchConfiguration("headless"),
            "debug": LaunchConfiguration("debug"),
            "verbose": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_name_1", default_value="robmini_01"),
        DeclareLaunchArgument("robot_name_2", default_value="robmini_02"),
        DeclareLaunchArgument("namespace_1", default_value=""),
        DeclareLaunchArgument("namespace_2", default_value=""),
        DeclareLaunchArgument("tf_prefix_1", default_value=""),
        DeclareLaunchArgument("tf_prefix_2", default_value=""),
        DeclareLaunchArgument("initial_pose_x_1", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y_1", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_z_1", default_value="0.078"),
        DeclareLaunchArgument("initial_pose_a_1", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_x_2", default_value="-0.8"),
        DeclareLaunchArgument("initial_pose_y_2", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_z_2", default_value="0.078"),
        DeclareLaunchArgument("initial_pose_a_2", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("sim_drive_mode", default_value="planar", choices=["ros2_control", "planar"]),
        DeclareLaunchArgument("map_file", default_value="room_mini/room_mini.yaml"),
        DeclareLaunchArgument("world_name", default_value=os.path.join(pkg_description, "worlds", "room_mini.world")),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("delay_spawn", default_value="4.0"),
        DeclareLaunchArgument("delay_bringup", default_value="10.0"),
        DeclareLaunchArgument("delay_rviz", default_value="14.0"),
        gazebo_launch,
        OpaqueFunction(function=_prepare_multi_robot_nodes),
    ])
