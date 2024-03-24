from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "is_station",
                default_value="False",
                description="Start bat in station or rover mode?",
            ),
            Node(
                package="kalman_rosou",
                executable="rosou_node",
                name="rosou",
                namespace="kalman_rover",
                output="screen",
                emulate_tty=True,
                parameters=[{"is_station": LaunchConfiguration("is_station")}],
            ),
        ]
    )
