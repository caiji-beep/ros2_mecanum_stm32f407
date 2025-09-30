#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, SetLaunchConfiguration
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import TimerAction



# ---------- ① 生成临时 YAML（带 initial_pose 嵌套写法） ----------
def _generate_temp_yaml(context):
    robot_name = context.launch_configurations["robot_name"]

    # 模板路径
    pkg_share = get_package_share_directory('robmini_navigation')
    template_path = os.path.join(pkg_share, "config", "nav2_params.yaml")

    with open(template_path, "r") as f:
        config = yaml.safe_load(f)

    # 递归替换 ${robot_name}
    def _replace(obj):
        if isinstance(obj, str):
            return obj.replace("${robot_name}", robot_name)
        if isinstance(obj, list):
            return [_replace(i) for i in obj]
        if isinstance(obj, dict):
            return {k: _replace(v) for k, v in obj.items()}
        return obj

    config = _replace(config)

    # ------ 修正 AMCL 初始位姿写法 ------
    amcl_param = config.setdefault('amcl', {}).setdefault('ros__parameters', {})

    # 清理旧散键
    for k in list(amcl_param):
        if k.startswith("initial_pose."):
            amcl_param.pop(k)

    # 嵌套格式（数值会在运行期被覆盖）
    amcl_param['set_initial_pose'] = True
    amcl_param.setdefault('initial_pose', {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0,
        'yaw': 0.0
    })

    # ------ 补全 DWB critics（保持你原来的逻辑） ------
    ctrl = config.get('controller_server', {}).get('ros__parameters', {})
    follow = ctrl.get('FollowPath', {})
    if 'critics' not in follow:
        follow['critics'] = [
            "RotateToGoal", "Oscillation", "BaseObstacle",
            "GoalAlign", "PathAlign", "PathDist", "GoalDist"
        ]

    # 写入临时文件
    fd, temp_path = tempfile.mkstemp(suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(config, f)

    return [SetLaunchConfiguration("nav2_params_path", temp_path)]


# ---------- ② 生成 LaunchDescription ----------
def generate_launch_description():
    # ---------------- 声明可调参数 ----------------
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')   # true=仿真, false=真机
    robot_name_arg   = DeclareLaunchArgument('robot_name',   default_value='robmini')
    initial_pose_x   = DeclareLaunchArgument('initial_pose_x', default_value='0.0')
    initial_pose_y   = DeclareLaunchArgument('initial_pose_y', default_value='0.0')
    initial_pose_a   = DeclareLaunchArgument('initial_pose_a', default_value='0.0')
    use_rviz_arg     = DeclareLaunchArgument('use_rviz',     default_value='false')
    map_file_arg     = DeclareLaunchArgument('map_file',     default_value='room_mini/115_map.yaml')

    # -------- 方便后续读取的 LaunchConfiguration --------
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name   = LaunchConfiguration('robot_name')
    init_x_arg   = LaunchConfiguration('initial_pose_x')
    init_y_arg   = LaunchConfiguration('initial_pose_y')
    init_a_arg   = LaunchConfiguration('initial_pose_a')
    use_rviz     = LaunchConfiguration('use_rviz')
    map_file     = LaunchConfiguration('map_file')
    nav2_params  = LaunchConfiguration('nav2_params_path')

    # 地图完整路径
    full_map = PathJoinSubstitution([
        get_package_share_directory('robmini_navigation'),
        'maps',
        map_file
    ])

    # ---------------- prepare_nodes ----------------
    def prepare_nodes(context):
        ns          = robot_name.perform(context)
        nav2_yaml   = context.launch_configurations['nav2_params_path']
        nav2_dir    = get_package_share_directory('nav2_bringup')
        nav2_dir_01    = get_package_share_directory('robmini_navigation')
        rviz_cfg    = PathJoinSubstitution([nav2_dir, 'rviz', 'nav2_default_view.rviz'])
        rviz_config_dir = os.path.join(nav2_dir_01, 'rviz', 'nav2.rviz') 


        # 解析 is_sim -> 布尔 use_sim_time（注意是布尔，不是字符串）
        use_sim_time_str = context.launch_configurations.get('use_sim_time', 'false')
        use_sim_time_bool = str(use_sim_time_str).lower() in ['1', 'true', 'yes']

        # 转换初始位姿为 float
        try:
            ix = float(init_x_arg.perform(context)); iy = float(init_y_arg.perform(context)); ia = float(init_a_arg.perform(context))
        except ValueError:
            ix = iy = ia = 0.0

        return [GroupAction([
            PushRosNamespace(robot_name),

            # ---------- Map Server ----------
            Node(
                package='nav2_map_server', executable='map_server',
                name='map_server', output='screen',
                parameters=[{
                    'yaml_filename': full_map.perform(context),
                    'frame_id': f"{ns}/map",
                    'use_sim_time': use_sim_time_bool
                }]
            ),

            # ---------- AMCL ----------
            Node(
                package='nav2_amcl', executable='amcl',
                name='amcl', output='screen',
                parameters=[
                    nav2_yaml,                               # 临时 YAML
                    {   # 运行期覆盖：嵌套 initial_pose
                        'set_initial_pose': True,
                        'initial_pose': {'x': ix, 'y': iy, 'z': 0.0, 'yaw': ia},
                        'use_map_topic': True,
                        'use_sim_time': use_sim_time_bool,
                        'base_frame_id':   f"{ns}/base_link",
                        'odom_frame_id':   f"{ns}/odom",
                        'global_frame_id': f"{ns}/map",
                        'scan_topic':      'scan'
                    }
                ]
            ),

            # ---------- Lifecycle Manager (localization) ----------
            Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager_localization', output='screen',
                parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]
            ),

            # ---------- Nav2 主 launch ----------
            IncludeLaunchDescription(
                PathJoinSubstitution([nav2_dir_01, 'launch', 'navigation_launch.py']),
                launch_arguments={
                    'params_file': nav2_yaml,
                    'autostart':  'true',
                    'use_sim_time': 'true' if use_sim_time_bool else 'false',
                    'namespace': ns
                }.items()
            ),

            # Node(
            #     package="robmini_navigation",
            #     executable="odom_bridge",
            #     name="odom_bridge",
            #     output="screen"
            # ),

            # ---------- RViz ----------
            # Node(
            #     condition=IfCondition(use_rviz),
            #     package='rviz2', executable='rviz2', name='rviz2',
            #     arguments=['-d', rviz_cfg.perform(context), '-f', f"{ns}/map"],
            #     parameters=[{'use_sim_time': True}]
            # )

            Node(
                condition=IfCondition(use_rviz),
                package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', rviz_config_dir, '-f', f"{ns}/map"],
                parameters=[{'use_sim_time': use_sim_time_bool}]
            )

        ])]

    # ---------------- LaunchDescription 返回 ----------------
    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg, 
        initial_pose_x, 
        initial_pose_y, 
        initial_pose_a,
        use_rviz_arg, 
        map_file_arg,
        OpaqueFunction(function=_generate_temp_yaml),
        OpaqueFunction(function=prepare_nodes)
    ])
