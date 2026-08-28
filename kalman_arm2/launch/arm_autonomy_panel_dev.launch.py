from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("kalman_arm2")
    bag_path = os.path.join(pkg_share, "rosbag2_2026_08_26-00_51_05")

    gs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("kalman_gs"),
                "launch",
                "gs.launch.py",
            )
        )
    )

    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--loop",
            "--remap",
            "/arm/panel/pixel_to_panel:=/arm/panel/homography",
        ],
        output="screen",
    )

    return LaunchDescription([bag_play, gs_launch])
