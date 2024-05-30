from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="pc",
                description="Master mode: 'pc', 'gs', or 'arm'.",
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
                    "drivers.rgbd_ids": "",
                    "drivers.master": "true",
                    "drivers.master.mode": LaunchConfiguration("mode"),
                    "drivers.imu": "false",
                    "drivers.compass_calibration": "false",
                    "drivers.gps": "false",
                    "wheel_controller": "true",
                }.items(),
            ),
        ]
    )
