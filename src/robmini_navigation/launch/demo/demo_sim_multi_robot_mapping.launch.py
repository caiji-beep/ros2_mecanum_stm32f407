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


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _as_float(value, default):
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return default


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
        "map_origin_x": context.launch_configurations.get(f"map_origin_x_{index}", "0.0"),
        "map_origin_y": context.launch_configurations.get(f"map_origin_y_{index}", "0.0"),
        "map_origin_yaw": context.launch_configurations.get(f"map_origin_yaw_{index}", "0.0"),
    }


def _prepare_multi_robot_mapping(context, *args, **kwargs):
    pkg_description = get_package_share_directory("robmini_description")
    pkg_navigation = get_package_share_directory("robmini_navigation")

    robot_1 = _robot_settings(context, 1)
    robot_2 = _robot_settings(context, 2)

    use_sim_time = context.launch_configurations.get("use_sim_time", "true")
    sim_drive_mode = context.launch_configurations.get("sim_drive_mode", "planar")
    use_rviz = context.launch_configurations.get("use_rviz", "true")
    target_frame = context.launch_configurations.get("target_frame", "world")
    merged_map_topic = context.launch_configurations.get("merged_map_topic", "/merged_map")
    publish_period = context.launch_configurations.get("merge_publish_period", "1.0")
    clear_start_regions = context.launch_configurations.get("clear_start_regions", "true")
    clear_start_radius = context.launch_configurations.get("clear_start_radius", "0.45")
    clear_robot_trails = context.launch_configurations.get("clear_robot_trails", "true")
    clear_robot_trail_radius = context.launch_configurations.get("clear_robot_trail_radius", "0.35")

    spawn_delay = _as_float(context.launch_configurations.get("delay_spawn", "4.0"), 4.0)
    mapping_delay = _as_float(context.launch_configurations.get("delay_mapping", "8.0"), 8.0)
    merge_delay = _as_float(context.launch_configurations.get("delay_merge", "12.0"), 12.0)
    rviz_delay = _as_float(context.launch_configurations.get("delay_rviz", "13.0"), 13.0)

    spawn_launch = os.path.join(pkg_description, "launch", "spawn_robot.launch.py")
    mapping_launch = os.path.join(pkg_navigation, "launch", "bringup_mapping_sim.launch.py")
    rviz_config = os.path.join(pkg_navigation, "rviz", "multi_robots_mapping.rviz")

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

    def start_mapping(robot):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                "robot_name": robot["robot_name"],
                "namespace": robot["namespace"],
                "tf_prefix": robot["tf_prefix"],
                "map_frame": robot["map_frame"],
                "use_sim_time": use_sim_time,
                "use_rviz": "false",
            }.items(),
        )

    def world_to_map_tf(robot):
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"world_to_{robot['tf_prefix'].replace('/', '_')}_map",
            arguments=[
                "--x",
                robot["map_origin_x"],
                "--y",
                robot["map_origin_y"],
                "--z",
                "0",
                "--yaw",
                robot["map_origin_yaw"],
                "--frame-id",
                target_frame,
                "--child-frame-id",
                robot["map_frame"],
            ],
        )

    map_merge_node = Node(
        package="robmini_navigation",
        executable="simple_map_merge.py",
        name="map_merge",
        output="screen",
        parameters=[{
            "use_sim_time": _as_bool(use_sim_time),
            "target_frame": target_frame,
            "merged_map_topic": merged_map_topic,
            "publish_period": float(publish_period),
            "resolution": 0.05,
            "map_topics": [
                f"/{robot_1['namespace']}/map",
                f"/{robot_2['namespace']}/map",
            ],
            "clear_regions": [
                f"{robot_1['initial_pose_x']},{robot_1['initial_pose_y']},{clear_start_radius}",
                f"{robot_2['initial_pose_x']},{robot_2['initial_pose_y']},{clear_start_radius}",
            ] if _as_bool(clear_start_regions) else [""],
            "clear_robot_frames": [
                _join_frame(robot_1["tf_prefix"], "base_link"),
                _join_frame(robot_2["tf_prefix"], "base_link"),
            ] if _as_bool(clear_robot_trails) else [""],
            "clear_robot_initial_poses": [
                f"{robot_1['initial_pose_x']},{robot_1['initial_pose_y']}",
                f"{robot_2['initial_pose_x']},{robot_2['initial_pose_y']}",
            ] if _as_bool(clear_robot_trails) else [""],
            "clear_robot_radius": float(clear_robot_trail_radius),
        }],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz_multi_robot_mapping",
        output="screen",
        arguments=["-d", rviz_config, "-f", target_frame],
        parameters=[{"use_sim_time": _as_bool(use_sim_time)}],
    )

    return [
        TimerAction(period=spawn_delay, actions=[spawn_robot(robot_1)]),
        TimerAction(period=spawn_delay + 1.0, actions=[spawn_robot(robot_2)]),
        TimerAction(period=mapping_delay, actions=[start_mapping(robot_1)]),
        TimerAction(period=mapping_delay + 1.0, actions=[start_mapping(robot_2)]),
        TimerAction(period=mapping_delay + 2.0, actions=[world_to_map_tf(robot_1)]),
        TimerAction(period=mapping_delay + 2.5, actions=[world_to_map_tf(robot_2)]),
        TimerAction(period=merge_delay, actions=[map_merge_node]),
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
        DeclareLaunchArgument("map_origin_x_1", default_value="0.0"),
        DeclareLaunchArgument("map_origin_y_1", default_value="0.0"),
        DeclareLaunchArgument("map_origin_yaw_1", default_value="0.0"),
        DeclareLaunchArgument("map_origin_x_2", default_value="0.0"),
        DeclareLaunchArgument("map_origin_y_2", default_value="0.0"),
        DeclareLaunchArgument("map_origin_yaw_2", default_value="0.0"),
        DeclareLaunchArgument("target_frame", default_value="world"),
        DeclareLaunchArgument("merged_map_topic", default_value="/merged_map"),
        DeclareLaunchArgument("merge_publish_period", default_value="1.0"),
        DeclareLaunchArgument("clear_start_regions", default_value="true"),
        DeclareLaunchArgument("clear_start_radius", default_value="0.45"),
        DeclareLaunchArgument("clear_robot_trails", default_value="true"),
        DeclareLaunchArgument("clear_robot_trail_radius", default_value="0.35"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("sim_drive_mode", default_value="planar", choices=["ros2_control", "planar"]),
        DeclareLaunchArgument("world_name", default_value=os.path.join(pkg_description, "worlds", "room_mini.world")),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("delay_spawn", default_value="4.0"),
        DeclareLaunchArgument("delay_mapping", default_value="8.0"),
        DeclareLaunchArgument("delay_merge", default_value="12.0"),
        DeclareLaunchArgument("delay_rviz", default_value="13.0"),
        gazebo_launch,
        OpaqueFunction(function=_prepare_multi_robot_mapping),
    ])
