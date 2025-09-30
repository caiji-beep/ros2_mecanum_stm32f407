import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 声明所有参数
    robot_name = DeclareLaunchArgument(
        'robot_name', default_value='robmini',
        description='Default robot namespace'
    )
    
    paused = DeclareLaunchArgument(
        'paused', default_value='false',
        description='Start Gazebo in paused state'
    )
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set to "false" to run gazebo headless'
    )
    
    headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Set to "true" to run gazebo headless'
    )
    
    debug = DeclareLaunchArgument(
        'debug', default_value='false',
        description='Debug mode for Gazebo'
    )
    
    world_name = DeclareLaunchArgument(
        'world_name', default_value=os.path.join(
            get_package_share_directory('robmini_description'),
            'worlds',
            'room_mini.world'
        ),
        description='Full path to world file to load'
    )

    # 获取机器人描述
    xacro_path = os.path.join(
        get_package_share_directory('robmini_description'),
        'urdf',
        'robmini_run.urdf.xacro'
    )
    
    robot_description = ParameterValue(
        Command([
            'xacro ', 
            xacro_path,
            ' prefix:=', 
            LaunchConfiguration('robot_name')
        ]),
        value_type=str
    )

    # 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('gazebo_ros'),
            'launch',
            'gazebo.launch.py'
        ),
        launch_arguments={
            'world': LaunchConfiguration('world_name'),
            'debug': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless'),
            'verbose': 'true',
            'server_required': 'false'
        }.items()
    )

    # 处理机器人命名空间下的所有组件
    robot_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('robot_name')),
        
        # 加载机器人描述
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 50.0,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
        ),
        
        # 关节状态发布器
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'rate': 50,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
        ),
        
        # 在 Gazebo 中生成机器人 - 关键修改点
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_entity',
                    output='screen',
                    arguments=[
                        '-topic', [LaunchConfiguration('robot_name'), '/robot_description'],  # 使用带命名空间的话题
                        '-entity', LaunchConfiguration('robot_name'),
                        '-robot_namespace', LaunchConfiguration('robot_name'),
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.0',
                        '-Y', '0.0'
                    ]
                )
            ]
        )
    ])

    return LaunchDescription([
        robot_name,
        paused,
        use_sim_time,
        gui,
        headless,
        debug,
        world_name,
        gazebo,
        robot_group
    ])