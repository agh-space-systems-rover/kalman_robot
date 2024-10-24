from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_path


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_bringup")
                        / "launch"
                        / "_commons.launch.py"
                    )
                ),
                launch_arguments={
                    "gs": "true",
                    "hardware": "true",
                    "hardware.master": "true",
                    "hardware.master.mode": "gs",
                    "hardware.master.drivers.arm": "true",
                    "hardware.master.drivers.feed": "true",
                    "hardware.master.drivers.drill": "true",
                    "wheels": "true",
                    "wheels.joy": "arduino",
                    "spacenav": "true",
                    "rviz": "true",
                    "rviz.config": "arm1 arm2 arm3",
                    "arm_utils": "true",
                    "description": "true",
                    "description.with_arm": "true",
                }.items(),
            ),
        ]
    )
