from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context):
    return [
        Node(
            package="kalman_rscp",
            executable="rscp_node",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
