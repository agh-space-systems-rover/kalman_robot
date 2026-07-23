from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ip_address",
                description="IP address of the RSCP TCP server.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="5555",
                description="TCP port of the RSCP TCP server.",
            ),
            Node(
                package="kalman_arc",
                executable="rscp_tcp_client",
                name="rscp_tcp_client",
                parameters=[
                    {
                        "host": LaunchConfiguration("ip_address"),
                        "port": ParameterValue(
                            LaunchConfiguration("port"),
                            value_type=int,
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )
