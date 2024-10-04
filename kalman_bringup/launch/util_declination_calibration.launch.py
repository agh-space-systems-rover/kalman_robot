from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    TimerAction,
    DeclareLaunchArgument,
)

from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "delay",
                default_value="3",
                description="Begin calibration after a delay. (seconds)",
            ),
            DeclareLaunchArgument(
                "duration",
                default_value="10",
                description="Keep driving forward for that long. (seconds)",
            ),
            DeclareLaunchArgument(
                "velocity",
                default_value="0.5",
                description="Driving speed. (m/s)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_bringup")
                        / "launch"
                        / "_commons.launch.py"
                    )
                ),
                launch_arguments={
                    "hardware": "true",
                    "hardware.master": "true",
                    "hardware.master.mode": "pc",
                    "hardware.imu": "true",
                    "hardware.declination_calibration": "true",
                    "hardware.declination_calibration.delay": LaunchConfiguration("delay"),
                    "hardware.declination_calibration.duration": LaunchConfiguration("duration"),
                    "hardware.declination_calibration.velocity": LaunchConfiguration("velocity"),
                    "hardware.gps": "true",
                    "wheels": "true",
                }.items(),
            ),
        ]
    )
