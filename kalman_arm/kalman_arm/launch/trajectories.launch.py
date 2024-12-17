from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pose_request_node = Node(
        package="kalman_arm_utils",
        executable="pose_request_sender",
    )

    trajectory_node = Node(
        package="kalman_arm_utils",
        executable="trajectory_sender",
    )

    return LaunchDescription([pose_request_node, trajectory_node])
