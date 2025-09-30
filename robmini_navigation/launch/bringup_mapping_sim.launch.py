"""
bringup_mapping.launch.py
=========================
Reusable SLAM‑mapping launch file that scales from **one** to **many** robots.

* Works on Foxy → Humble → Iron (no RewrittenYaml dependency).
* One parameter — ``robot_name`` — decides every namespace / TF prefix.
* Generates a **temporary YAML** from a template containing ``${robot_name}`` placeholders.
* Remaps the three hard‑coded absolute map topics produced by *slam_toolbox*.

Directory layout (example)
└── robot_navigation2/
    ├── launch/
    │   └── bringup_mapping.launch.py   ← (this file)
    └── config/
        └── slam_template.yaml          ← see README or earlier chat

Usage
-----
# Robot 1
ros2 launch robot_navigation2 bringup_mapping.launch.py robot_name:=f219bot_01

# Robot 2 (second terminal)
ros2 launch robot_navigation2 bringup_mapping.launch.py robot_name:=f219bot_02

Map topics become /f219bot_01/map and /f219bot_02/map respectively.
"""

import os
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

# ----------------------------------------------------------------------------
# Helper: generate a temporary YAML with ${robot_name} expanded
# ----------------------------------------------------------------------------

def _gen_temp_yaml(context, *args, **kwargs):
    robot_name = context.launch_configurations["robot_name"]

    # Resolve template path inside this package
    pkg_share = FindPackageShare("robmini_navigation").perform(context)
    template_path = os.path.join(pkg_share, "config", "slam_template.yaml")

    # Load template
    with open(template_path, "r") as f:
        data = yaml.safe_load(f)

    # Recursively substitute ${robot_name}
    def _subst(obj):
        if isinstance(obj, str):
            return obj.replace("${robot_name}", robot_name)
        if isinstance(obj, list):
            return [_subst(v) for v in obj]
        if isinstance(obj, dict):
            return {k: _subst(v) for k, v in obj.items()}
        return obj

    data = _subst(data)

    # Write to a secure temporary file
    fd, tmp_path = tempfile.mkstemp(suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f)

    # Expose path to rest of launch as substitution "slam_yaml"
    return [SetLaunchConfiguration("slam_yaml", tmp_path)]


# ----------------------------------------------------------------------------
# LaunchDescription factory
# ----------------------------------------------------------------------------

def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    slam_yaml = LaunchConfiguration("slam_yaml")  # set by _gen_temp_yaml

    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox", 
        parameters=[slam_yaml, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/map",          ["/", robot_name, "/map"]),
            ("/map_metadata", ["/", robot_name, "/map_metadata"]),
            ("/map_updates",  ["/", robot_name, "/map_updates"]),
        ],
        output="screen",
    )

        # ---------------- RViz 2 ----------------
    pkg_share = FindPackageShare("robmini_navigation")

        # ——  RViz2 可视化 —— #
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz', output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'bringup_mapping.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(
        [
            # ---------------- Launch arguments ----------------
            DeclareLaunchArgument(
                "robot_name",
                default_value="robmini",
                description="Namespace & TF prefix for this robot",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),

            # -------------- Runtime YAML generation ------------
            OpaqueFunction(function=_gen_temp_yaml),

            # -------------- Namespaced node group --------------
            GroupAction(
                actions=[
                    PushRosNamespace(robot_name),
                    slam_node,
                    rviz_node,
                ]
            ),
        ]
    )
