from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    TimerAction,
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import json


def launch_setup(context):
    delay = float(LaunchConfiguration("delay").perform(context))
    two_d = LaunchConfiguration("two_d").perform(context).lower() == "true"
    duration = float(LaunchConfiguration("duration").perform(context))
    angular_velocity = float(LaunchConfiguration("angular_velocity").perform(context))

    req_json_str = json.dumps(
        {
            "two_d": two_d,
            "duration": duration,
            "angular_velocity": angular_velocity,
        }
    )

    return [
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
                "drivers.composition": "false",
                "drivers.rgbd_ids": "",
                "drivers.master": "true",
                "drivers.master.mode": "pc",
                "drivers.imu": "false",
                "drivers.compass_calibration": "true",
                "drivers.gps": "false",
                "wheel_controller": "true",
            }.items(),
        ),
        # Call the calibration service
        TimerAction(
            period=delay,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "service",
                        "call",
                        "/compass_calibration/calibrate",
                        "kalman_interfaces/srv/CalibrateCompass",
                        req_json_str,
                    ],
                )
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "delay",
                default_value="3",
                description="Call the calibration service after a delay. (seconds)",
            ),
            DeclareLaunchArgument(
                "two_d",
                default_value="true",
                description="Only calibrate yaw.",
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
            OpaqueFunction(function=launch_setup),
        ]
    )
