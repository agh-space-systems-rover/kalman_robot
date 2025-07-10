from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode


def node_or_component(
    component_container,
    package,
    executable,
    plugin=None,
    name=None,
    namespace=None,
    parameters=[],
    remappings=[],
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
                        extra_arguments=[{"use_intra_process_comms": True}],
                        **kwargs
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
                **kwargs
            ),
        ]


def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)
    
    actions = []

    # Joint republisher
    actions += node_or_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="joint_republisher",
        plugin="kalman_arm2::JointRepublisher",
        namespace="arm",
        remappings=[
            ("current_pos", "joints/current_pos"),
            ("joint_states", "joint_states"),
        ]
    )

    # TODO: Add twist_ik_node when implemented

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "component_container",
            default_value="",
            description="Name of an existing component container to use. Empty to disable composition.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
