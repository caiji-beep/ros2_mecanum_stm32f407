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
    block_size = LaunchConfiguration("block_size")
    ack_timeout_ms = LaunchConfiguration("ack_timeout_ms")
    capability_timeout_ms = LaunchConfiguration("capability_timeout_ms")
    receive_timeout_ms = LaunchConfiguration("receive_timeout_ms")
    max_retries = LaunchConfiguration("max_retries")
    inter_frame_delay_us = LaunchConfiguration("inter_frame_delay_us")

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("node_id", default_value="1"),
            DeclareLaunchArgument("request_base_id", default_value="1536"),
            DeclareLaunchArgument("response_base_id", default_value="1408"),
            DeclareLaunchArgument("extended_id", default_value="false"),
            DeclareLaunchArgument("block_size", default_value="256"),
            DeclareLaunchArgument("ack_timeout_ms", default_value="1000"),
            DeclareLaunchArgument("capability_timeout_ms", default_value="1000"),
            DeclareLaunchArgument("receive_timeout_ms", default_value="100"),
            DeclareLaunchArgument("max_retries", default_value="3"),
            DeclareLaunchArgument("inter_frame_delay_us", default_value="0"),
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
                        "block_size": ParameterValue(block_size, value_type=int),
                        "ack_timeout_ms": ParameterValue(ack_timeout_ms, value_type=int),
                        "capability_timeout_ms": ParameterValue(
                            capability_timeout_ms, value_type=int
                        ),
                        "receive_timeout_ms": ParameterValue(receive_timeout_ms, value_type=int),
                        "max_retries": ParameterValue(max_retries, value_type=int),
                        "inter_frame_delay_us": ParameterValue(
                            inter_frame_delay_us, value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
