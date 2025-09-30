# robmini_description/launch/real_bringup.launch.py
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('robot_name', default_value='robmini'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_mock', default_value='false'),
    ]

    robot_name   = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = os.path.join(
        get_package_share_directory('robmini_description'),
        'urdf', 'robmini_run.urdf.xacro'
    )
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, 
                 ' prefix:=', LaunchConfiguration('robot_name'),
                 ' use_mock:=', LaunchConfiguration('use_mock')]),
        value_type=str
    )

    # 控制器参数：用 mecanum_drive_controller
    # ctrl_yaml = PathJoinSubstitution([
    #     get_package_share_directory('robmini_description'),
    #     'config', 'robmini_mecanum_controllers.yaml'
    # ])
    ctrl_yaml = ParameterFile(
    PathJoinSubstitution([get_package_share_directory('robmini_description'),
                          'config', 'robmini_mecanum_controllers.yaml']),
    allow_substs=True  # 允许使用 $(var ...) 进行变量替换
)

    # 启动 ros2_control_node（真机必需）
    controller_mgr = GroupAction([
        PushRosNamespace(LaunchConfiguration('robot_name')),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': robot_description}, 
                        ctrl_yaml, 
                        #{'use_sim_time': LaunchConfiguration('use_sim_time')}
                        ],
            remappings=[
                ('mecanum_drive_controller_test01/reference_unstamped', 'cmd_vel'),
                ('mecanum_drive_controller_test01/odometry', 'odom'),
                ('mecanum_drive_controller_test01/tf_odometry', '/tf'),  # /tf 保持绝对
                ],
            # remappings=[
            #     # 让控制器订阅 /robmini/cmd_vel（unstamped）
            #     ('/robmini/mecanum_drive_controller_test01/reference_unstamped', '/robmini/cmd_vel'),
            #     # 让控制器发布里程计到 /robmini/odom 和 /tf
            #     ('/robmini/mecanum_drive_controller_test01/odometry', '/robmini/odom'),
            #     ('/robmini/mecanum_drive_controller_test01/tf_odometry', '/tf'),
            # ],
        ),
        # robot_state_publisher（发布 TF/关节状态）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 
                         'publish_frequency': 50.0,
                         'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        # 等待 controller_manager ready 再 spawn 控制器
        TimerAction(
            period=2.0,
            actions=[
                Node(package='controller_manager', executable='spawner',
                     arguments=['joint_broad_test01', '--controller-manager', ['/', LaunchConfiguration('robot_name'), '/controller_manager']])
            ]),
        TimerAction(
            period=3.0,
            actions=[
                Node(package='controller_manager', executable='spawner',
                     arguments=['mecanum_drive_controller_test01', '--controller-manager', ['/', LaunchConfiguration('robot_name'), '/controller_manager']])
            ]),
    ])

    ld = LaunchDescription(declared_arguments)
    ld.add_action(controller_mgr)
    return ld
