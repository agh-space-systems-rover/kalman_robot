from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python import get_package_share_path


def launch_setup(context):
    actions = []

    actions += [
        Node(
            package="kalman_health",
            executable="topic_health_monitor",
            parameters=[
                {
                    "config_path": str(
                        get_package_share_path("kalman_health")
                        / "config"
                        / "monitors.yaml"
                    ),
                }
            ],
        ),
    ]

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
