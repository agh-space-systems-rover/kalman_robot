from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "composition",
                default_value="false",
                description="Use node composition where applicable.",
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
                    "description": "true",
                    "description.joint_state_publisher_gui": "false",
                    "slam": "true",
                    "slam.composition": LaunchConfiguration("composition"),
                    "slam.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "slam.gps": "true",
                    "slam.gps_datum": "50.8780616423677 20.642475756324",  # Marsyard S1, Kielce
                    "slam.no_gps_map_odom_offset": "0 0",  # Assuming that the stack will be enabled at S1
                    "slam.mapping": "false",
                    "nav2": "true",
                    "nav2.composition": LaunchConfiguration("composition"),
                    "nav2.rgbd_ids": "",
                    "nav2.static_map": "erc2023",
                    "wheel_controller": "true",
                    # "aruco": "true",
                    # "aruco.composition": LaunchConfiguration("composition"),
                    # "aruco.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    # "yolo": "true",
                    # "yolo.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    # "yolo.config": "urc2024",
                }.items(),
            ),
        ]
    )
