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
    actions += [
        Node(
            package="kalman_arm2",
            executable="joint_republisher",
            namespace="arm",
            remappings=[
                ("current_pos", "current_pos"),
                ("joint_states", "joint_states"),
            ],
        )
    ]

    # Twist IK node
    actions += node_or_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="twist_ik",
        plugin="kalman_arm2::TwistIK",
        namespace="arm",
        remappings=[
            ("current_pos", "current_pos"),
            ("target_twist", "target_twist"),
            ("target_vel", "target_vel/joints"),
        ],
    )

    # Gamepad control node
    actions += [
        Node(
            package="kalman_arm2",
            executable="gamepad_control",
            namespace="arm",
            remappings=[
                ("joy", "/joy"),
                ("target_twist", "target_twist"),
                ("jaw_vel", "target_vel/jaw"),
            ],
        )
    ]

    actions += node_or_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="goto_joint_pose",
        plugin="kalman_arm2::GotoJointPose",
        namespace="arm",
        remappings=[
            ("current_pos", "current_pos"),
            ("target_vel", "target_vel"),
            *remap_action("goto_pose", "goto_pose"),
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

    # Share topics from RPi
    for topic, msg_type, mode in [
        ("/arm_controllers/joint_states", "sensor_msgs/msg/JointState", "recv"),
        ("/servo_node/delta_joint_cmds", "control_msgs/msg/JointJog", "send"),
        ("/gripper/position", "std_msgs/msg/UInt16", "recv"),
        ("/gripper/command_absolute", "std_msgs/msg/UInt16", "send"),
        ("/gripper/command_incremental", "std_msgs/msg/Int8", "send"),
    ]:
        actions += [
            Node(
                package="kalman_arm2",
                executable="rosbridge_client",
                name="rosbridge_client_" + topic.replace("/", "_").strip("_"),
                parameters=[
                    {
                        "ws_address": "192.168.2.77:9473",
                        "topic": topic,
                        "type": msg_type,
                        "mode": mode,
                    }
                ],
            )
        ]

    # Translate legacy API
    actions += [
        Node(
            package="kalman_arm2",
            executable="arm1_moveit_compat",
            namespace="arm",
            remappings=[
                ("new/current_pos", "current_pos"),
                ("new/target_pos/jaw", "target_pos/jaw"),
                ("new/target_vel", "target_vel"),
                ("new/target_vel/joints", "target_vel/joints"),
                ("new/target_vel/jaw", "target_vel/jaw"),
                ("old/arm_controllers/joint_states", "/arm_controllers/joint_states"),
                ("old/servo_node/delta_joint_cmds", "/servo_node/delta_joint_cmds"),
                ("old/gripper/position", "/gripper/position"),
                ("old/gripper/command_absolute", "/gripper/command_absolute"),
                ("old/gripper/command_incremental", "/gripper/command_incremental"),
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
