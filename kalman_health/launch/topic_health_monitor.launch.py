from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_path


def launch_setup(context):
    description = []

    description += [
        Node(
            package="kalman_health",
            executable="topic_health_monitor",
            parameters=[
                {
                    "config_path": str(
                        get_package_share_path("kalman_health")
                        / f"config/monitored_topics.yaml"
                    ),
                }
            ]
        ),
    ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value="",
                description="Config that contains topics to monitor"
            ),
            OpaqueFunction(function=launch_setup)
        ]
    )
