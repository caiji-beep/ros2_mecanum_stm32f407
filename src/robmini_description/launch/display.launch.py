import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明机器人名称参数
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robmini',
        description='Name of the robot'
    )
    
    # 声明模型文件参数
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
            get_package_share_directory('robmini_description'),
            'urdf',
            'robmini_run.urdf.xacro'  # 默认URDF文件
        ]),
        description='Path to robot URDF file'
    )

    # 获取 robot_description 包的路径
    pkg_robmini = get_package_share_directory('robmini_description')
    
    # 包含模型启动文件
    model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_robmini, 
                'launch', 
                'includes', 
                'model.launch.py'  # 使用您之前转换的 model.launch.py
            ])
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'model': LaunchConfiguration('model')  # 传递model参数
        }.items()
    )

    
    # 启动 RViz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=[
            '-d', PathJoinSubstitution([
                pkg_robmini, 
                'rviz', 
                'model_01.rviz'  # 保持原文件名不变
            ]),
            '-f', [LaunchConfiguration('robot_name'), '/base_link']
        ]
    )
    
    return LaunchDescription([
        robot_name_arg,
        model_arg,
        model_launch,
        rviz_node
    ])