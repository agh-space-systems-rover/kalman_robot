import os

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

from kalman_bringup.utils import get_arg_decls, is_kalman_composable

# If type checking, import the right TypedDict instead of using regular dict
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from kalman_bringup.type_hints import BringupConfig
else:
    BringupConfig = dict

COMPONENT_CONTAINER_NAME = "kalman_container"


def gen_launch(config: BringupConfig, composition=False) -> LaunchDescription:
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

    for module_name, args in config.items():
        if module_name not in installed_packages:
            module_pkg = f"kalman_{module_name}"
        else:
            module_pkg = module_name

        # Get the launch file path
        try:
            launch_path = str(
                get_package_share_path(module_pkg)
                / "launch"
                / f"{module_name}.launch.py"
            )
        except PackageNotFoundError:
            raise ValueError(f"Package {module_pkg} not found")

        if not os.path.exists(launch_path):
            raise ValueError(f"Launch file {launch_path} not found")

        # Add component_container argument if composition is enabled
        launch_arguments = args.items()
        if composition and is_kalman_composable(launch_path):
            launch_arguments += [("component_container", COMPONENT_CONTAINER_NAME)]

        # Verify if arguments are valid:
        arg_decls = get_arg_decls(launch_path)
        for key, value in launch_arguments:
            if key not in [x[0] for x in arg_decls]:
                raise ValueError(
                    f'Invalid argument "{key}" for "{module_name}". '
                    f"Valid arguments are: {[x[0] for x in arg_decls]}. "
                    f"Argument description: {desc}"
                )

            choices = [x[2] for x in arg_decls if x[0] == key]
            choices = choices[0] if choices else None
            if choices and value not in choices:
                raise ValueError(
                    f'Invalid choice "{value}" for argument "{key}". '
                    f"Valid choices are: {choices}. "
                    f"Argument description: {desc}"
                )

        # Include the launch file
        desc.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_path),
                launch_arguments=launch_arguments,
            )
        )

    return LaunchDescription(desc)
