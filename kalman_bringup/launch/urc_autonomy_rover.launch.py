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
                    "component_container": "true",
                    "description": "true",
                    "description.joint_state_publisher_gui": "false",
                    "drivers": "true",
                    "drivers.composition": "true",
                    "drivers.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "drivers.master": "true",
                    "drivers.imu": "true",
                    "drivers.compasscal": "false",
                    "drivers.gps": "true",
                    "slam": "true",
                    "slam.composition": "true",
                    "slam.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "slam.gps": "false",
                    "slam.gps_datum": "",  # automatic = no initial map->odom offset
                    "slam.no_gps_map_odom_offset": "0 0",
                    "slam.mapping": "false",
                    "nav2": "true",
                    "nav2.composition": "true",
                    "nav2.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "nav2.static_map": "",
                    "wheel_controller": "true",
                    "aruco": "true",
                    "aruco.composition": "true",
                    "aruco.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "yolo": "true",
                    "yolo.rgbd_ids": "d455_front",
                    "yolo.config": "urc2024",
                    "supervisor": "true",
                }.items(),
            ),
        ]
    )
