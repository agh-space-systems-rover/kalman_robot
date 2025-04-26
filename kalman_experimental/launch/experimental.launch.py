from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context):

    description = [
        Node(
            package="kalman_experimental",
            executable="tag_republisher",

        )
    ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
