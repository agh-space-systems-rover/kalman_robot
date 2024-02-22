from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_drivers")
                        / "launch"
                        / "drivers.launch.py"
                    )
                ),
                launch_arguments={
                    "rgbd_ids": "",
                    "imu": "false",
                    "compasscal": "true"
                }.items(),
            ),
            Node(
                package="kalman_wheel_controller",
                executable="wheel_controller",
                parameters=[
                    {
                        "robot_radius": 0.5,
                    }
                ],
            ),
        ]
    )
