#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_gazebo_ros2_control.launch.py
Gazebo + ros2_control 仿真（spawn_entity 延时 5 s；控制器时序随之顺延）
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    GroupAction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ----------- 通用参数 -----------
    declared_arguments = [
        DeclareLaunchArgument('robot_name', default_value='robmini',
                              description='机器人命名空间/实体名'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('robmini_description'),
                'worlds', 'room_mini.world')),
        DeclareLaunchArgument('gui',      default_value='true'),
        DeclareLaunchArgument('paused',   default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug',    default_value='false'),
    ]

    # 延时常量（秒）
    SPAWN_DELAY = 5.0            # spawn_entity 延时
    JS_DELAY    = SPAWN_DELAY + 5.0   # joint_state_broadcaster
    MEC_DELAY   = SPAWN_DELAY + 7.0   # mecanum_controller

    # ----------- 机器人描述 -----------
    xacro_file = os.path.join(
        get_package_share_directory('robmini_description'),
        'urdf', 'robmini_run.urdf.xacro')

    robot_description = ParameterValue(
        Command([
            'xacro', ' ', xacro_file,
            ' prefix:=', LaunchConfiguration('robot_name')
        ]),
        value_type=str)
    
    robot_controller_config = os.path.join(get_package_share_directory('robmini_description'), 'config', 'robmini_controller.yaml')

    # ----------- Gazebo 本体 -----------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world':     LaunchConfiguration('world'),
            'gui':       LaunchConfiguration('gui'),
            'paused':    LaunchConfiguration('paused'),
            'headless':  LaunchConfiguration('headless'),
            'debug':     LaunchConfiguration('debug'),
            'verbose':   'true'
        }.items())

    # ----------- 机器人实体 + TF -----------
    robot_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('robot_name')),

        # robot_state_publisher 立即启动
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen',
             parameters=[{
                 'robot_description': robot_description,
                 'publish_frequency': 50.0,
                 'use_sim_time':      LaunchConfiguration('use_sim_time')
             }]),

        # spawn_entity 延迟 5 s
        TimerAction(
            period=SPAWN_DELAY,
            actions=[
                Node(package='gazebo_ros',
                     executable='spawn_entity.py',
                     arguments=[
                         '-topic',
                         [LaunchConfiguration('robot_name'),
                          '/robot_description'],
                         '-entity', LaunchConfiguration('robot_name'),
                         '-robot_namespace', LaunchConfiguration('robot_name'),
                         '-x', '0', '-y', '0', '-z', '0.055'
                     ],
                     output='screen')
            ]),
    ])

    # ----------- 加载控制器 -----------
    controller_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('robot_name')),

        # joint_state_broadcaster
        TimerAction(
            period=JS_DELAY,
            actions=[
                Node(package='controller_manager',
                     executable='spawner',
                     arguments=[
                         "joint_broad_test01",
                         '--controller-manager',
                         ['/', LaunchConfiguration('robot_name'), '/controller_manager']
                     ],
                     output='screen')
            ]),

        # mecanum_controller
        TimerAction(
            period=MEC_DELAY,
            actions=[
                Node(package='controller_manager',
                     executable='spawner',
                     arguments=[
                         "mecanum_drive_controller_test01",
                         '--controller-manager',
                         ['/', LaunchConfiguration('robot_name'), '/controller_manager']
                     ],
                     output='screen')
            ]),  

    ])

    # ----------- LaunchDescription -----------
    ld = LaunchDescription(declared_arguments)
    ld.add_action(gazebo_launch)
    ld.add_action(robot_group)
    ld.add_action(controller_group)
    return ld
