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
                    "description.joint_state_publisher_gui": "false",
                    "clouds": "true",
                    "clouds.composition": "false",
                    "clouds.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "slam": "true",
                    "slam.composition": "false",
                    "slam.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "slam.gps": "true",
                    "slam.gps_datum": "50.8780616423677 20.642475756324",  # Marsyard S1, Kielce
                    "slam.no_gps_map_odom_offset": "0 0",  # Assuming that the stack will be enabled at S1
                    "slam.mapping": "false",
                    "nav2": "true",
                    "nav2.composition": "false",
                    "nav2.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "nav2.static_map": "",
                    "wheel_controller": "true",
                    "aruco": "true",
                    "aruco.composition": "false",
                    "aruco.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "yolo": "true",
                    "yolo.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "yolo.config": "urc2024",
                    "supervisor": "true",
                }.items(),
            ),
        ]
    )
