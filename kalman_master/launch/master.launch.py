from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial port to connect to",
            ),
            Node(
                package="kalman_master",
                executable="master_com",
                name="master_com",
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                    }
                ],
            ),
            Node(
                package="kalman_master",
                executable="wheel_driver",
            ),
            Node(
                package="kalman_master",
                executable="ueuos_driver",
            ),
        ]
    )
