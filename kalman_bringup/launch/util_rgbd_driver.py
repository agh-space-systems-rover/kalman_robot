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
                "composition",
                default_value="false",
                description="Use node composition where applicable.",
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                default_value="",
                description="Comma separated list of RGBD camera IDs to enable.",
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
                    "component_container": LaunchConfiguration("composition"),
                    "drivers": "true",
                    "drivers.composition": LaunchConfiguration("composition"),
                    "drivers.rgbd_ids": LaunchConfiguration("rgbd_ids"),
                    "drivers.master": "false",
                    "drivers.imu": "false",
                    "drivers.compasscal": "false",
                }.items(),
            ),
        ]
    )
