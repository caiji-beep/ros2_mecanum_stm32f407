from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    node_id = LaunchConfiguration("node_id")
    request_base_id = LaunchConfiguration("request_base_id")
    response_base_id = LaunchConfiguration("response_base_id")
    extended_id = LaunchConfiguration("extended_id")
    default_firmware_path = LaunchConfiguration("default_firmware_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("node_id", default_value="1"),
            DeclareLaunchArgument("request_base_id", default_value="1536"),
            DeclareLaunchArgument("response_base_id", default_value="1408"),
            DeclareLaunchArgument("extended_id", default_value="false"),
            DeclareLaunchArgument(
                "default_firmware_path",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), ".stm32_can_ota", "firmware", "latest.bin"]
                ),
            ),
            Node(
                package="stm32_can_ota",
                executable="stm32_ota_node",
                name="stm32_ota_node",
                output="screen",
                parameters=[
                    {
                        "can_interface": can_interface,
                        "node_id": ParameterValue(node_id, value_type=int),
                        "request_base_id": ParameterValue(request_base_id, value_type=int),
                        "response_base_id": ParameterValue(response_base_id, value_type=int),
                        "extended_id": ParameterValue(extended_id, value_type=bool),
                        "default_firmware_path": default_firmware_path,
                    }
                ],
            ),
        ]
    )
