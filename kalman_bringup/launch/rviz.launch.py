from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    str(
                        get_package_share_path("kalman_bringup")
                        / "rviz"
                        / "default.rviz"
                    ),
                ],
            ),
        ]
    )
