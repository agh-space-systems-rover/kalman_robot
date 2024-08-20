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
                default_value="30",
                description="Continue rotation for this duration. (seconds)",
            ),
            DeclareLaunchArgument(
                "angular_velocity",
                default_value="0.5",
                description="Rotation speed. (rad/s)",
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
                    "drivers": "true",
                    "drivers.master": "true",
                    "drivers.master.mode": "pc",
                    "drivers.compass_calibration": "true",
                    "drivers.compass_calibration.delay": LaunchConfiguration("delay"),
                    "drivers.compass_calibration.duration": LaunchConfiguration("duration"),
                    "drivers.compass_calibration.angular_velocity": LaunchConfiguration("angular_velocity"),
                    "wheel_controller": "true",
                }.items(),
            ),
        ]
    )
