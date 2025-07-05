import os
import glob

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import ExecuteProcess

from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    data_path = LaunchConfiguration("data_path").perform(context)
    data_path = os.path.normpath(os.path.abspath(os.path.expanduser(data_path)))

    # Recursively find the latest trajectory-*.yaml file in data_path, sort by name.
    files = sorted(
        glob.glob(os.path.join(data_path, "**", "trajectory-*.yaml"), recursive=True)
    )
    if len(files) == 0:
        raise RuntimeError(f"No trajectory-*.yaml files found in {data_path}")
    trajectory_yaml_path = files[-1]

    # Define other dirs.
    cube_dir = os.path.join(data_path, "cube")
    cube_confirmed_dir = os.path.join(data_path, "cube-confirmed")

    # Find the publisher executable.
    cube_viewer = str(get_package_share_path("kalman_arch") / "tools" / "cube_viewer")

    return [
        ExecuteProcess(
            cmd=[
                cube_viewer,
                cube_dir,
                trajectory_yaml_path,
                cube_confirmed_dir,
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
