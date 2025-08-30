from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from kalman_utils.launch import launch_node_or_load_component, remap_action


def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)

    actions = []

    # Joint republisher
    actions += launch_node_or_load_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="joint_republisher",
        plugin="kalman_arm2::JointRepublisher",
        namespace="arm",
        remappings=[
            ("current_pos", "current_pos"),
            ("joint_states", "joint_states"),
        ],
    )

    # Twist IK node
    actions += launch_node_or_load_component(
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

    actions += launch_node_or_load_component(
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

    panel_layout_file = PathJoinSubstitution(
        [FindPackageShare("kalman_arm2"), "config", "panel_layout.yaml"]
    )

    tree_xml_file = PathJoinSubstitution(
        [FindPackageShare("kalman_arm2"), "trees", "demo.xml"]
    )

    actions += launch_node_or_load_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="bt_panel",
        plugin="kalman_arm2::BTPanel",
        namespace="arm",
        remappings=[
            ("current_pos", "current_pos"),
            ("target_vel", "target_vel"),
            *remap_action("goto_pose", "goto_pose"),
        ],
        parameters=[
            {"layout_yaml": ParameterValue(panel_layout_file, value_type=str)},
            {"tree_xml": ParameterValue(tree_xml_file, value_type=str)},
            {"auto_start": ParameterValue(False, value_type=bool)},
            # {"auto_start": ParameterValue(True, value_type=bool)},
        ],
    )

    actions += launch_node_or_load_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="ik_navigate_to_pose",
        plugin="kalman_arm2::IKNavigateToPoseActionServer",
        namespace="arm",
        remappings=[
            ("current_pos", "current_pos"),
            ("target_vel", "target_vel"),
            *remap_action("goto_pose", "goto_pose"),
        ],
        # parameters=[
        #     {"layout_yaml": ParameterValue(panel_layout_file, value_type=str)},
        #     {"tree_xml": ParameterValue(tree_xml_file, value_type=str)},
        #     {"auto_start": ParameterValue(False, value_type=bool)},
        #     # {"auto_start": ParameterValue(True, value_type=bool)},
        # ],
    )

    actions += launch_node_or_load_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="panel_layout",
        plugin="kalman_arm2::PanelLayout",
        namespace="arm",
        remappings=[
            ("current_pos", "current_pos"),
            ("target_vel", "target_vel"),
            *remap_action("goto_pose", "goto_pose"),
        ],
        parameters=[{"layout_yaml": ParameterValue(panel_layout_file, value_type=str)}],
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
        ("/joy_compressed", "kalman_interfaces/msg/ArmCompressed", "send"),
    ]:
        break
        actions += [
            Node(
                package="kalman_arm2",
                executable="rosbridge_client",
                name="rosbridge_client_" + topic.replace("/", "_").strip("_"),
                parameters=[
                    {
                        "ws_address": "192.168.1.2:9473",
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
                ("old/joy_compressed", "/joy_compressed"),
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
