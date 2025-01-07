from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_path

# for testing new yaml configs

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="kalman_master",
                executable="universal_driver",
                parameters=[
                    {
                        "config_directory_path": str(
                            get_package_share_path("kalman_master")
                            / "config/universal_driver"
                        ),
                    },
                ],
            ),
        ]
    )
