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
    **kwargs,
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
                        **kwargs,
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
        ],
    )

    # Twist IK node
    actions += node_or_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="twist_ik",
        plugin="kalman_arm2::TwistIK",
        namespace="arm",
        remappings=[
            ("current_pos", "joints/current_pos"),
            ("target_twist", "ik/target_twist"),
            ("target_vel", "joints/target_vel"),
        ],
    )

    # Gamepad control node
    actions += node_or_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="gamepad_control",
        plugin="kalman_arm2::GamepadControl",
        namespace="arm",
        remappings=[
            ("joy", "/joy"),
            ("target_twist", "ik/target_twist"),
            ("jaw_vel", "joints/target_vel"),
        ],
    )


    actions += node_or_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="goto_joint_pose",
        plugin="kalman_arm2::GotoJointPose",
        namespace="arm",
        remappings=[
            ("current_pos", "joints/current_pos"),
            ("target_vel", "joints/target_vel"),
            *remap_action("goto_pose", "joints/goto_pose"),
        ],
    )

    # Joy node
    actions += [
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            parameters=[
                {
                    "dev_name": "Logitech Gamepad",
                }
            ],
        )
    ]

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="",
                description="Name of an existing component container to use. Empty to disable composition.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
