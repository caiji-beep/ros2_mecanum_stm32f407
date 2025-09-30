"""
bringup_mapping_real.launch.py - 真机建图启动文件
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
    IncludeLaunchDescription,  # 用于包含其他启动文件
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 

def _gen_temp_yaml(context, *args, **kwargs):
    robot_name = context.launch_configurations["robot_name"]

    pkg_share = FindPackageShare("robmini_navigation").perform(context)
    template_path = os.path.join(pkg_share, "config", "slam_template.yaml")  


    with open(template_path, "r") as f:
        data = yaml.safe_load(f)

    def _subst(obj):
        if isinstance(obj, str):
            return obj.replace("${robot_name}", robot_name)
        if isinstance(obj, list):
            return [_subst(v) for v in obj]
        if isinstance(obj, dict):
            return {k: _subst(v) for k, v in obj.items()}
        return obj

    data = _subst(data)

    fd, tmp_path = tempfile.mkstemp(suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f)

    return [SetLaunchConfiguration("slam_yaml", tmp_path)]

def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")#真机false
    slam_yaml = LaunchConfiguration("slam_yaml")

    # # 【新增】键盘控制节点 - 真机专用
    # teleop_node = Node(
    #     package="robmini_navigation",  # 替换为您的包名
    #     executable="mecanum_teleop_keyboard",
    #     name="mecanum_teleop_keyboard",
    #     output="screen",
    #     prefix="xterm -e",  # 在新终端中运行
    #     parameters=[{
    #         "linear_speed": 0.15,    # 真机建议较低速度
    #         "angular_speed": 0.3,
    #         "strafe_speed": 0.15,
    #     }],
    #     remappings=[
    #         # 【修改】根据您的真机控制器话题调整
    #         ("/cmd_vel", ["/", robot_name, "/cmd_vel"])
    #     ]
    # )

    # 【修改】SLAM节点参数 - 使用async版本更适合真机
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",  # 【修改】sync → async
        name="slam_toolbox", 
        parameters=[slam_yaml, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/map", ["/", robot_name, "/map"]),
            ("/map_metadata", ["/", robot_name, "/map_metadata"]),
            ("/map_updates", ["/", robot_name, "/map_updates"]),
        ],
        output="screen",
    )

    # 【保持不变】RViz2节点
    pkg_share = FindPackageShare("robmini_navigation")
    rviz_node = Node(
        package='rviz2', 
        executable='rviz2', 
        name='rviz', 
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'bringup_mapping.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 静态变换节点 - 真机可能需要
    # 如果您的激光雷达与base_link有固定偏移，需要此节点
    # static_transform_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_link_to_laser',
    #     arguments=[
    #         '0.1', '0', '0.1', '0', '0', '0',  # x, y, z, roll, pitch, yaw
    #         [robot_name, '/base_link'], 
    #         [robot_name, '/laser']  # 根据您的激光雷达frame_id调整
    #     ],
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    return LaunchDescription(
        [
            # 【保持不变】启动参数声明
            DeclareLaunchArgument(
                "robot_name",
                default_value="robmini",
                description="Namespace & TF prefix for this robot",
            ),
            # 【修改】use_sim_time 默认值从 "true" 改为 "false"
            DeclareLaunchArgument(
                "use_sim_time", 
                default_value="false",
                description="Use simulation (Gazebo) clock if true"
            ),

            # 【保持不变】YAML生成
            OpaqueFunction(function=_gen_temp_yaml),

            # 【修改】节点组 - 添加了键盘控制和静态变换
            GroupAction(
                actions=[
                    PushRosNamespace(robot_name),
                    slam_node,
                    # teleop_node,           # 【新增】键盘控制
                    # static_transform_node, # 【新增】静态变换
                    # rviz_node,
                ]
            ),
        ]
    )