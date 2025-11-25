import yaml
import os

from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_path

def launch_node_or_load_component(
    component_container,
    package,
    executable,
    plugin=None,
    name=None,
    namespace=None,
    parameters=[],
    remappings=[],
    extra_arguments=[{"use_intra_process_comms": True}],
    **kwargs
):
    if name is None:
        name = executable

    if component_container:
        if plugin is None:
            raise ValueError(f"Plugin name required for composable node {name}")

        return [
            LoadComposableNodes(
                target_container=component_container,
                composable_node_descriptions=[
                    ComposableNode(
                        package=package,
                        plugin=plugin,
                        name=name,
                        namespace=namespace,
                        parameters=parameters,
                        remappings=remappings,
                        extra_arguments=extra_arguments,
                    ),
                ],
            ),
        ]
    else:
        return [
            Node(
                package=package,
                executable=executable,
                name=name,
                namespace=namespace,
                parameters=parameters,
                remappings=remappings,
                **kwargs,
            ),
        ]
    
def remap_action(from_name, to_name):
    return [
        (f"{from_name}/_action/send_goal", f"{to_name}/_action/send_goal"),
        (f"{from_name}/_action/cancel_goal", f"{to_name}/_action/cancel_goal"),
        (f"{from_name}/_action/feedback", f"{to_name}/_action/feedback"),
        (f"{from_name}/_action/get_result", f"{to_name}/_action/get_result"),
        (f"{from_name}/_action/status", f"{to_name}/_action/status"),
    ]

def load_standalone_config(pkg: str, yaml_filename: str) -> dict:
    yaml_path = str(get_package_share_path(pkg) / "config" / yaml_filename)
    return load_standalone_config_file(yaml_path)

def load_standalone_config_file(yaml_path: str) -> dict:
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"YAML file not found: {yaml_path}")

    # Load the yaml
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    
    # Recursively find first ros__parameters key and return its value
    def find_ros_parameters(data):
        if isinstance(data, dict):
            if "ros__parameters" in data:
                return data["ros__parameters"]
            for value in data.values():
                result = find_ros_parameters(value)
                if result is not None:
                    return result
        return None
    
    # Find the first ros__parameters key
    ros_parameters = find_ros_parameters(data)
    if ros_parameters is not None:
        return ros_parameters
    else:
        raise ValueError(f"No 'ros__parameters' key found in {yaml_path}")
