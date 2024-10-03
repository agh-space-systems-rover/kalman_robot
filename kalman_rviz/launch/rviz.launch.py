from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    def get_str(name):
        return LaunchConfiguration(name).perform(context)

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                str(
                    get_package_share_path("kalman_rviz")
                    / "rviz"
                    / get_str("rviz.config")
                ),
                "--ros-args",
                "--log-level",
                "warn",
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="",
                description="RViz configuration file name without extension, e.g. autonomy",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
