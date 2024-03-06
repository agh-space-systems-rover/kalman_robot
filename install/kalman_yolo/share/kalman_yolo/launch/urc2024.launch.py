from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="kalman_yolo",
                executable="yolo_detect",
                parameters=[
                    ParameterFile(
                        str(
                            get_package_share_path("kalman_yolo")
                            / "param"
                            / "urc2024.yaml"
                        ),
                        allow_substs=True,
                    )
                ],
            ),
        ]
    )
