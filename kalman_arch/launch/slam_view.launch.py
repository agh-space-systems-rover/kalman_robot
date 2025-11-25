import os
import glob

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    data_path = LaunchConfiguration("data_path").perform(context)
    data_path = os.path.normpath(os.path.abspath(os.path.expanduser(data_path)))

    # Recursively find the latest cloud-*.ply file in data_path, sort by name.
    files = sorted(
        glob.glob(os.path.join(data_path, "**", "cloud-*.ply"), recursive=True)
    )
    if len(files) == 0:
        raise RuntimeError(f"No cloud-*.ply files found in {data_path}")
    cloud_ply_path = files[-1]

    # Recursively find the latest trajectory-*.yaml file in data_path, sort by name.
    files = sorted(
        glob.glob(os.path.join(data_path, "**", "trajectory-*.yaml"), recursive=True)
    )
    if len(files) == 0:
        raise RuntimeError(f"No trajectory-*.yaml files found in {data_path}")
    trajectory_yaml_path = files[-1]

    # Find the publisher executable.
    map_and_trajectory_sender = str(
        get_package_share_path("kalman_arch") / "tools" / "map_and_trajectory_sender"
    )

    return [
        ExecuteProcess(
            cmd=[
                map_and_trajectory_sender,
                "--ply",
                cloud_ply_path,
                "--yaml",
                trajectory_yaml_path,
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                str(get_package_share_path("kalman_arch") / "rviz" / "slam_view.rviz"),
                "--ros-args",
                "--log-level",
                "warn",
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "data_path",
                default_value="~/arch",
                description="Path to the directory that contains cloud and trajectory subfolders.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
