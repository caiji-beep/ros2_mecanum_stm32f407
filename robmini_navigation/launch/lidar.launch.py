#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明新参数
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robmini',
        description='Name of the robot'
    )
    
    laser_frame_arg = DeclareLaunchArgument(
        'laser_frame',
        default_value=[LaunchConfiguration('robot_name'), '/laser'],
        description='Frame ID for the laser scanner'
    )
    
    # 声明原始rplidar的所有参数
    channel_type_arg = DeclareLaunchArgument(
        'channel_type',
        default_value='serial',
        description='Specifying channel type of lidar'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyRadar',
        description='Specifying usb port to connected lidar'
    )
    
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Specifying usb port baudrate to connected lidar'
    )
    
    inverted_arg = DeclareLaunchArgument(
        'inverted',
        default_value='false',
        description='Specifying whether or not to invert scan data'
    )
    
    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Specifying whether or not to enable angle_compensate of scan data'
    )
    
    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='Specifying scan mode of lidar'
    )
    
    # 获取原始rplidar launch文件路径
    rplidar_launch_path = PathJoinSubstitution([
        FindPackageShare('rplidar_ros'),
        'launch',
        'rplidar_a1_launch.py'
    ])
    
    # 创建雷达节点（直接创建而不是包含launch）
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        namespace=LaunchConfiguration('robot_name'),  # 添加命名空间
        parameters=[{
            'channel_type': LaunchConfiguration('channel_type'),
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': LaunchConfiguration('laser_frame'),
            'inverted': LaunchConfiguration('inverted'),
            'angle_compensate': LaunchConfiguration('angle_compensate'),
            'scan_mode': LaunchConfiguration('scan_mode')
        }],
        output='screen',
        # 自动添加前缀到所有话题
        remappings=[
            ('scan', 'scan')  # 话题变为 /<robot_name>/scan
        ]
    )

    return LaunchDescription([
        # 新参数
        robot_name_arg,
        laser_frame_arg,
        
        # 原始参数声明
        channel_type_arg,
        serial_port_arg,
        serial_baudrate_arg,
        inverted_arg,
        angle_compensate_arg,
        scan_mode_arg,
        
        # 直接启动雷达节点
        rplidar_node
    ])