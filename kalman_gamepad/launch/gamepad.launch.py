from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="kalman_gamepad",
                executable="gamepad",
                parameters=[
                    str(
                        get_package_share_path("kalman_gamepad")
                        / "config"
                        / "gamepad.yaml"
                    )
                ],
            ),
        ]
    )
