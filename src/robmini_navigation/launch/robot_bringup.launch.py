#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file robot_bringup.launch.py
@brief RobMini 机器人 Nav2 导航系统启动文件

该 launch 文件用于启动 RobMini 机器人的 Nav2 导航相关节点，主要包括：
1. map_server：加载并发布静态地图；
2. amcl：基于激光雷达和地图进行机器人定位；
3. lifecycle_manager_localization：管理 map_server 和 amcl 的生命周期；
4. navigation_launch.py：启动 Nav2 规划、控制、行为树等导航模块；
5. rviz2：可选启动 RViz2，用于可视化地图、TF、路径和导航状态。

该文件支持以下典型运行场景：
- 单机器人导航；
- 多机器人命名空间隔离；
- 自定义 TF 前缀；
- 自定义地图文件；
- 自定义初始位姿；
- 仿真环境或实机环境切换；
- 可选启动 RViz2 调试界面。
"""

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _as_auto_bool(value, auto_value):
    text = str(value).strip().lower()
    if text in ("", "auto"):
        return bool(auto_value)
    return _as_bool(text)


def _clean_namespace(value):
    value = str(value or "").strip()
    if value == "/":
        return ""
    return value.strip("/")


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


def _set_key_recursive(obj, key_name, value):
    if isinstance(obj, dict):
        for key, item in obj.items():
            if key == key_name:
                obj[key] = value
            else:
                _set_key_recursive(item, key_name, value)
    elif isinstance(obj, list):
        for item in obj:
            _set_key_recursive(item, key_name, value)


def _nested_params(config, *keys):
    node = config
    for key in keys:
        if not isinstance(node, dict):
            return {}
        node = node.setdefault(key, {})
    return node if isinstance(node, dict) else {}


def _apply_low_load_nav2(config):
    amcl = _nested_params(config, "amcl", "ros__parameters")
    amcl.update({
        "max_beams": 40,
        "min_particles": 500,
        "max_particles": 2000,
        "resample_interval": 2,
    })

    bt = _nested_params(config, "bt_navigator", "ros__parameters")
    bt["bt_loop_duration"] = 20

    controller = _nested_params(config, "controller_server", "ros__parameters")
    controller["controller_frequency"] = 10.0

    follow_path = controller.setdefault("FollowPath", {})
    follow_path.update({
        "max_vel_theta": 0.65,
        "max_speed_theta": 0.65,
        "acc_lim_x": 0.8,
        "acc_lim_theta": 1.4,
        "decel_lim_x": -0.8,
        "decel_lim_theta": -1.4,
        "vx_samples": 12,
        "vtheta_samples": 16,
        "sim_time": 1.2,
        "transform_tolerance": 0.3,
    })

    local_costmap = _nested_params(config, "local_costmap", "local_costmap", "ros__parameters")
    local_costmap.update({
        "update_frequency": 5.0,
        "publish_frequency": 2.0,
        "always_send_full_costmap": False,
    })

    global_costmap = _nested_params(config, "global_costmap", "global_costmap", "ros__parameters")
    global_costmap.update({
        "update_frequency": 1.0,
        "publish_frequency": 0.5,
        "always_send_full_costmap": False,
    })

    behavior = _nested_params(config, "behavior_server", "ros__parameters")
    behavior["cycle_frequency"] = 5.0

    waypoint = _nested_params(config, "waypoint_follower", "ros__parameters")
    waypoint["loop_rate"] = 10

    velocity_smoother = _nested_params(config, "velocity_smoother", "ros__parameters")
    velocity_smoother.update({
        "smoothing_frequency": 10.0,
        "max_velocity": [0.22, 0.0, 0.65],
        "min_velocity": [-0.11, 0.0, -0.65],
        "max_accel": [0.8, 0.0, 1.4],
        "max_decel": [-0.8, 0.0, -1.4],
        "odom_duration": 0.3,
    })


def _write_nav2_yaml(template_path, namespace, tf_prefix, map_frame, odom_topic, low_load_nav2):
    with open(template_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    config = _replace_placeholders(config, {
        "${namespace}": namespace,
        "${tf_prefix}": tf_prefix,
        "${base_frame}": _join_frame(tf_prefix, "base_link"),
        "${odom_frame}": _join_frame(tf_prefix, "odom"),
        "${map_frame}": map_frame,
    })

    amcl_param = config.setdefault("amcl", {}).setdefault("ros__parameters", {})
    for key in list(amcl_param):
        if key.startswith("initial_pose."):
            amcl_param.pop(key)

    amcl_param["set_initial_pose"] = True
    amcl_param.setdefault("initial_pose", {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "yaw": 0.0,
    })

    controller_params = config.setdefault("controller_server", {}).setdefault("ros__parameters", {})
    controller_params["odom_topic"] = odom_topic

    follow_path = controller_params.setdefault("FollowPath", {})
    follow_path.setdefault("critics", [
        "RotateToGoal",
        "Oscillation",
        "BaseObstacle",
        "GoalAlign",
        "PathAlign",
        "PathDist",
        "GoalDist",
    ])

    _set_key_recursive(config, "odom_topic", odom_topic)

    if low_load_nav2:
        _apply_low_load_nav2(config)

    fd, temp_path = tempfile.mkstemp(prefix="robmini_nav2_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, sort_keys=False)

    direct_node_config = {namespace: config} if namespace else config
    fd, direct_temp_path = tempfile.mkstemp(prefix="robmini_nav2_direct_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        yaml.safe_dump(direct_node_config, f, sort_keys=False)

    return temp_path, direct_temp_path


def _set_rviz_display_enabled(display, enabled):
    display["Enabled"] = enabled
    display["Value"] = enabled


def _apply_low_load_rviz(data):
    root = data.get("Visualization Manager", {})
    root.setdefault("Global Options", {})["Frame Rate"] = 10

    disabled_names = {
        "TF",
        "LaserScan",
        "Amcl Particle Swarm",
        "Global Costmap",
        "Local Costmap",
        "Polygon",
        "MarkerArray",
    }

    def visit(displays):
        for display in displays or []:
            name = display.get("Name", "")
            display_class = display.get("Class", "")

            if name == "RobotModel":
                display["Update Interval"] = 0.2

            if name in disabled_names or display_class == "rviz_default_plugins/TF":
                _set_rviz_display_enabled(display, False)

            if display_class == "rviz_default_plugins/LaserScan":
                _set_rviz_display_enabled(display, False)

            visit(display.get("Displays", []))

    visit(root.get("Displays", []))
    return data


def _write_rviz_config(template_path, namespace, tf_prefix, map_frame, low_load_rviz):
    with open(template_path, "r", encoding="utf-8") as f:
        text = f.read()

    namespace_topic = f"/{namespace}" if namespace else ""
    tf_prefix_text = f"{tf_prefix}/" if tf_prefix else ""
    default_map_frame = _join_frame(tf_prefix, "map")

    text = text.replace("/robmini", namespace_topic)
    text = text.replace("robmini/", tf_prefix_text)
    if default_map_frame != map_frame:
        text = text.replace(f"Fixed Frame: {default_map_frame}", f"Fixed Frame: {map_frame}")
        text = text.replace(f"        {default_map_frame}:", f"        {map_frame}:")
        text = text.replace(f"      {default_map_frame}:", f"      {map_frame}:")

    if low_load_rviz:
        text = yaml.safe_dump(_apply_low_load_rviz(yaml.safe_load(text)), sort_keys=False)

    fd, temp_path = tempfile.mkstemp(prefix="robmini_rviz_", suffix=".rviz")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(text)
    return temp_path


def _prepare_nodes(context, *args, **kwargs):
    robot_name = context.launch_configurations.get("robot_name", "robmini")
    namespace = _clean_namespace(context.launch_configurations.get("namespace", ""))
    if not namespace:
        namespace = _clean_namespace(robot_name)

    tf_prefix = _clean_namespace(context.launch_configurations.get("tf_prefix", ""))
    if not tf_prefix:
        tf_prefix = namespace

    map_frame = context.launch_configurations.get("map_frame", "").strip()
    if not map_frame:
        map_frame = _join_frame(tf_prefix, "map")

    use_sim_time = context.launch_configurations.get("use_sim_time", "false")
    use_sim_time_bool = _as_bool(use_sim_time)
    use_ekf = context.launch_configurations.get("use_ekf", "false")
    use_ekf_bool = _as_bool(use_ekf)
    filtered_odom_topic = context.launch_configurations.get("filtered_odom_topic", "odometry/filtered").strip()
    odom_topic = filtered_odom_topic if use_ekf_bool else "odom"
    low_load_nav2 = _as_auto_bool(
        context.launch_configurations.get("low_load_nav2", "auto"),
        use_ekf_bool,
    )
    low_load_rviz = _as_auto_bool(
        context.launch_configurations.get("low_load_rviz", "auto"),
        low_load_nav2,
    )

    try:
        initial_x = float(context.launch_configurations.get("initial_pose_x", "0.0"))
        initial_y = float(context.launch_configurations.get("initial_pose_y", "0.0"))
        initial_a = float(context.launch_configurations.get("initial_pose_a", "0.0"))
    except ValueError:
        initial_x = initial_y = initial_a = 0.0

    pkg_share = get_package_share_directory("robmini_navigation")
    nav2_yaml, direct_nav2_yaml = _write_nav2_yaml(
        os.path.join(pkg_share, "config", "nav2_params.yaml"),
        namespace,
        tf_prefix,
        map_frame,
        odom_topic,
        low_load_nav2,
    )

    map_file = context.launch_configurations.get("map_file", "room_mini/115_map.yaml")
    if os.path.isabs(map_file):
        full_map = map_file
    else:
        full_map = os.path.join(pkg_share, "maps", map_file)

    rviz_config = _write_rviz_config(
        os.path.join(pkg_share, "rviz", "nav2.rviz"),
        namespace,
        tf_prefix,
        map_frame,
        low_load_rviz,
    )

    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            namespace=namespace,
            output="screen",
            parameters=[{
                "yaml_filename": full_map,
                "frame_id": map_frame,
                "use_sim_time": use_sim_time_bool,
            }],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            namespace=namespace,
            output="screen",
            parameters=[
                direct_nav2_yaml,
                {
                    "set_initial_pose": True,
                    "initial_pose": {
                        "x": initial_x,
                        "y": initial_y,
                        "z": 0.0,
                        "yaw": initial_a,
                    },
                    "use_map_topic": True,
                    "use_sim_time": use_sim_time_bool,
                    "base_frame_id": _join_frame(tf_prefix, "base_link"),
                    "odom_frame_id": _join_frame(tf_prefix, "odom"),
                    "global_frame_id": map_frame,
                    "scan_topic": "scan",
                },
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            namespace=namespace,
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["map_server", "amcl"],
                "use_sim_time": use_sim_time_bool,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "namespace": namespace,
                "params_file": nav2_yaml,
                "autostart": "true",
                "use_sim_time": "true" if use_sim_time_bool else "false",
            }.items(),
        ),
        Node(
            condition=IfCondition(context.launch_configurations.get("use_rviz", "false")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=namespace,
            output="screen",
            arguments=["-d", rviz_config, "-f", map_frame],
            parameters=[{"use_sim_time": use_sim_time_bool}],
            remappings=[
                ("tf", "/tf"),
                ("tf_static", "/tf_static"),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robmini"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("map_frame", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_a", default_value="0.0"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("map_file", default_value="room_mini/115_map.yaml"),
        DeclareLaunchArgument("use_ekf", default_value="false"),
        DeclareLaunchArgument("filtered_odom_topic", default_value="odometry/filtered"),
        DeclareLaunchArgument("low_load_nav2", default_value="auto"),
        DeclareLaunchArgument("low_load_rviz", default_value="auto"),
        OpaqueFunction(function=_prepare_nodes),
    ])
