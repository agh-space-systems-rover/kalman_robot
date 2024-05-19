from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
                    # "rviz": "true",
                    # "rviz.config": "urc_autonomy.rviz",
                    "drivers": "true",
                    "drivers.composition": "false",
                    "drivers.rgbd_ids": "",
                    "drivers.master": "true", # WARNING: Having two master drivers enabled on the same network will cause messages on topics shared via RF to loop around over network and cause a positive feedback loop! Remember to disable master here or disconnect it physically when connecting to the robot over network.
                    "drivers.master.mode": "gs",
                    "drivers.imu": "false",
                    "drivers.compass_calibration": "false",
                    "drivers.gps": "false",
                    "gs": "true",
                    "wheel_controller": "true",
                }.items(),
            ),
        ]
    )
