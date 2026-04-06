from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明可配置参数
    robot_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robmini',
        description='机器人命名空间名称'
    )

    # 构建带参数的终端命令
    teleop_cmd = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 
            'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard', 
            '--ros-args', 
            '--remap', ['/cmd_vel:=/', LaunchConfiguration('robot_name'), '/cmd_vel']
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_arg,
        teleop_cmd
    ])





