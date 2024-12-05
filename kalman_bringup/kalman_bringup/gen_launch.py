import os
import importlib.util

from ament_index_python import (
    get_package_share_path,
    PackageNotFoundError,
    get_search_paths,
)
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

COMPONENT_CONTAINER_NAME = "kalman_container"


# Extract arguments declared in a launch file
def get_arg_decls(launch_file: str):
    # Import generate_launch_description() from the specified launch file
    import_name = launch_file.split("/share/")[-1].replace("/", ".")[:-10]
    spec = importlib.util.spec_from_file_location(f"{import_name}", launch_file)
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)

    # Generate the launch description
    launch_desc: LaunchDescription = launch_module.generate_launch_description()

    # Extract the arguments from the launch description
    args: list[tuple[str, str]] = []  # name: desc
    for action in launch_desc.get_launch_arguments():
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
    installed_packages = [x.split("/")[-1] for x in get_search_paths()]

    if composition:
        # Spawn the component container
        desc += [
            Node(
                package="rclcpp_components",
                executable="component_container_mt",
                name=COMPONENT_CONTAINER_NAME,
                # arguments=["--ros-args", "--log-level", "warn"],
            )
        ]

    for module, args in config.items():
        module_name = module
        if module not in installed_packages:
            module_name = f"kalman_{module}"

        # Get the launch file path
        try:
            launch_path = str(
                get_package_share_path(module_name) / "launch" / f"{module}.launch.py"
            )
        except PackageNotFoundError:
            print(f"Package {module_name} not found")
            continue

        if not os.path.exists(launch_path):
            print(f"Launch file {launch_path} not found")
            continue

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
