from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    description = []

    description += [
        Node(
            package="kalman_telegram",
            executable="telegram_channel"
        ),
    ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup)
        ]
    )
