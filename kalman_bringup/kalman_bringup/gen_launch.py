import os

from ament_index_python import get_package_share_path, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

COMPONENT_CONTAINER_NAME = "kalman_container"


# Extract arguments declared in a launch file
def get_arg_decls(launch_file: str):
    # Import generate_launch_description() from the specified launch file
    gld = __import__(launch_file).generate_launch_description

    # Generate the launch description
    launch_desc = gld()

    # Extract the arguments from the launch description
    args: list[tuple[str, str]] = []  # name: desc
    for action in launch_desc.actions:
        if isinstance(action, DeclareLaunchArgument):
            args.append((action.name, action.description))

    return args


# See if a kalman_ launch file is composable
def is_kalman_composable(launch_file: str):
    args = get_arg_decls(launch_file)

    # Check if there's a component_container arg
    for arg_name, _ in args:
        if arg_name == "component_container":
            return True


def gen_launch(
    config: dict[str, dict[str, str]], composition=False
) -> LaunchDescription:
    desc = []

    if composition:
        # Spawn the component container
        description += [
            Node(
                package="rclcpp_components",
                executable="component_container_mt",
                name=COMPONENT_CONTAINER_NAME,
                # arguments=["--ros-args", "--log-level", "warn"],
            )
        ]

    for module, args in config.items():
        # Get the launch file path
        try:
            launch_path = str(
                get_package_share_path(f"kalman_{module}")
                / "launch"
                / f"{module}.launch.py"
            )
        except (PackageNotFoundError):
            launch_path = str(
                get_package_share_path(module) / "launch" / f"{module}.launch.py"
            )

        # Add component_container argument if composition is enabled
        launch_arguments = args.items()
        if composition:
            launch_arguments += [("component_container", COMPONENT_CONTAINER_NAME)]

        # Include the launch file
        desc.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_path),
                launch_arguments=launch_arguments,
            )
        )

    return LaunchDescription(desc)
