from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    configs = [
        x for x in LaunchConfiguration("config").perform(context).split(" ") if x != ""
    ]

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                str(get_package_share_path("kalman_rviz") / "rviz" / f"{config}.rviz"),
                "--ros-args",
                "--log-level",
                "warn",
            ],
        )
        for config in configs
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="",
                description="Space separated RViz configuration file names without extensions, e.g. 'autonomy demo_rgbd'",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
