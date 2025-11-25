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
    yaml_path = LaunchConfiguration("yaml_path").perform(context)
    yaml_path = os.path.normpath(os.path.abspath(os.path.expanduser(yaml_path)))

    # Recursively find the latest cloud-*.ply file in data_path, sort by name.
    files = sorted(
        glob.glob(os.path.join(data_path, "**", "cloud-*.ply"), recursive=True)
    )
    if len(files) == 0:
        raise RuntimeError(f"No cloud-*.ply files found in {data_path}")
    cloud_ply_path = files[-1]

    # Find the publisher executable.
    cloud_rasterizer = str(
        get_package_share_path("kalman_arch") / "tools" / "cloud_rasterizer"
    )

    cmd = f"{cloud_rasterizer} --ply {cloud_ply_path} --yaml {yaml_path} --out ~/arch/flatmap.png --fill-holes 3 && xdg-open ~/arch/flatmap.png"

    return [
        ExecuteProcess(
            cmd=[
                "/usr/bin/bash",
                "-c",
                cmd,
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
            DeclareLaunchArgument(
                "yaml_path",
                default_value="~/../arch.yaml",
                description="Path to the directory that contains cloud and trajectory subfolders.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
