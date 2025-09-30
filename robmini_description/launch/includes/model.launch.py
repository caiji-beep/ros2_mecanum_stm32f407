import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明参数 - 完全对应原文件的 <arg> 标签
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        description='Name of the robot'  # 无默认值，与原文件一致
    )
    
     # 添加缺失的 use_sim_time 声明
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
            get_package_share_directory('robmini_description'),
            'urdf',
            'robmini_run.urdf.xacro'  # 添加默认值
        ]),
        description='Path to robot URDF file'
    )
    
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='False',  # 与原文件 default="False" 一致
        description='Enable joint_state_publisher GUI'
    )
    
    # 获取 robot_description 包的路径
    pkg_f219bot = get_package_share_directory('robmini_description')
    
    # 加载机器人描述 - 完全复制 ROS 1 行为
    robot_description = Command([
        'xacro', ' ',
        LaunchConfiguration('model'), ' ',  # 对应原文件的 $(arg model)
        'prefix:=', LaunchConfiguration('robot_name')  # 对应原文件的 prefix:=$(arg robot_name)
    ])

    # 关节状态发布器节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'rate': 50,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # 机器人状态发布器节点 - 关键修复
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    
    return LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,
        model_arg,
        gui_arg,
        joint_state_publisher_node,
        robot_state_publisher_node
    ])