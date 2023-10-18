import os
from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription(
        [
            # Nav2
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_nav2")
                        / "launch"
                        / "navigation_launch.launch.py"
                    )
                ),
                launch_arguments={
                    "params_file": str(
                        get_package_share_path("kalman_nav2") / "param" / "nav2.yaml"
                    ),
                }.items(),
            ),
            # path follower
            Node(
                package="kalman_nav2",
                executable="path_follower",
                parameters=[
                    str(
                        get_package_share_path("kalman_nav2")
                        / "param"
                        / "path_follower.yaml"
                    ),
                ],
            ),
        ]
    )
