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
                    # "component_container": "true",
                    "description": "true",
                    "drivers": "true",
                    "drivers.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "drivers.master": "true",
                    "drivers.master.mode": "pc",
                    "drivers.imu": "true",
                    "clouds": "true",
                    "clouds.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "slam": "true",
                    "slam.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "slam.gps_datum": "50.06623534 19.9132241",  # ERC2024 Marsyard S1, Kraków
                    "nav2": "true",
                    "nav2.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "nav2.static_map": "erc2024",
                    "wheel_controller": "true",
                    "aruco": "true",
                    "aruco.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "yolo": "true",
                    "yolo.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "yolo.config": "urc2024",
                    "supervisor": "true",
                }.items(),
            ),
        ]
    )