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
            # DeclareLaunchArgument(
            #     "mode",
            #     default_value="gs",
            #     choices=["gs", "pc", "arm"],
            #     description="On what hardware is this module being run? Available modes: gs, pc, arm",
            # ),
            # DeclareLaunchArgument(
            #     "mode",
            #     default_value="gs",
            #     choices=["gs", "pc", "arm"],
            #     description="On what hardware is this module being run? Available modes: gs, pc, arm",
            # ),
            OpaqueFunction(function=launch_setup)
        ]
    )
