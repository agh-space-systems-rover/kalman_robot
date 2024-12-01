from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="kalman_arm_utils",
                executable="joint_republisher",
                name="joint_republisher",
            ),
            Node(
                package="kalman_arm_utils",
                executable="gripper_republisher",
                parameters=[{"gripper_scale": 25.0}],
            ),
            Node(
                package="kalman_arm_utils",
                executable="arm_state_republisher",
            ),
        ]
    )
