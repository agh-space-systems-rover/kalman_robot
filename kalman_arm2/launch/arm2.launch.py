import yaml
from kalman_utils.launch import launch_node_or_load_component, remap_action
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)
    twist_ik_params = {
        "base_damping": float(LaunchConfiguration("ik_base_damping").perform(context)),
        "max_damping": float(LaunchConfiguration("ik_max_damping").perform(context)),
        "singularity_sigma_threshold": float(
            LaunchConfiguration("ik_singularity_sigma_threshold").perform(context)
        ),
        "joint_centering_gain": float(
            LaunchConfiguration("ik_joint_centering_gain").perform(context)
        ),
        "enable_singularity_logging": yaml.safe_load(
            LaunchConfiguration("ik_enable_singularity_logging").perform(context)
        ),
        "singularity_log_period_ms": float(
            LaunchConfiguration("ik_singularity_log_period_ms").perform(context)
        ),
        "joint_motion_weights": yaml.safe_load(
            LaunchConfiguration("ik_joint_motion_weights").perform(context)
        ),
        "singularity_avoidance_gains": yaml.safe_load(
            LaunchConfiguration("ik_singularity_avoidance_gains").perform(context)
        ),
        "singularity_avoidance_thresholds": yaml.safe_load(
            LaunchConfiguration("ik_singularity_avoidance_thresholds").perform(context)
        ),
        "singularity_preferred_positions": yaml.safe_load(
            LaunchConfiguration("ik_singularity_preferred_positions").perform(context)
        ),
    }

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
        parameters=[twist_ik_params],
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

    panel_tracker_params = {
        "tracking_frame": LaunchConfiguration("panel_tracking_frame").perform(context),
        "board_frame": LaunchConfiguration("panel_board_frame").perform(context),
        "detection_topic": LaunchConfiguration("panel_detection_topic").perform(
            context
        ),
        "ema_alpha": float(
            LaunchConfiguration("panel_tracker_ema_alpha").perform(context)
        ),
    }

    if 1:
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
            ],
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

    actions += launch_node_or_load_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="panel_tracker",
        plugin="kalman_arm2::PanelTracker",
        namespace="arm",
        parameters=[panel_tracker_params],
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
            DeclareLaunchArgument(
                "ik_base_damping",
                default_value="0.03",
                description="Base damping factor for twist IK.",
            ),
            DeclareLaunchArgument(
                "ik_max_damping",
                default_value="0.35",
                description="Maximum damping factor for twist IK near singularities.",
            ),
            DeclareLaunchArgument(
                "ik_singularity_sigma_threshold",
                default_value="0.12",
                description="Smallest singular value threshold below which damping ramps up.",
            ),
            DeclareLaunchArgument(
                "ik_joint_centering_gain",
                default_value="0.35",
                description="Nullspace gain that pulls joints toward preferred positions.",
            ),
            DeclareLaunchArgument(
                "ik_enable_singularity_logging",
                default_value="true",
                description="Enable throttled twist IK singularity diagnostics.",
            ),
            DeclareLaunchArgument(
                "ik_singularity_log_period_ms",
                default_value="1000.0",
                description="Throttle period for twist IK diagnostics in milliseconds.",
            ),
            DeclareLaunchArgument(
                "ik_joint_motion_weights",
                default_value="[1.0, 1.0, 1.0, 10.0, 0.5, 1.0]",
                description="Per-joint motion weights for weighted pseudoinverse IK.",
            ),
            DeclareLaunchArgument(
                "ik_singularity_avoidance_gains",
                default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 1.2]",
                description="Per-joint nullspace bias gains used near singular configurations.",
            ),
            DeclareLaunchArgument(
                "ik_singularity_avoidance_thresholds",
                default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.35]",
                description="Per-joint activation thresholds for singularity avoidance bias.",
            ),
            DeclareLaunchArgument(
                "ik_singularity_preferred_positions",
                default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 1.4]",
                description="Per-joint preferred positions used by singularity avoidance bias.",
            ),
            DeclareLaunchArgument(
                "panel_tracking_frame",
                default_value="base_link",
                description="Frame in which the panel tracker estimates the board pose.",
            ),
            DeclareLaunchArgument(
                "panel_board_frame",
                default_value="aruco_board",
                description="TF child frame published by the panel tracker.",
            ),
            DeclareLaunchArgument(
                "panel_detection_topic",
                default_value="/d455_arm/aruco_detections",
                description="Aruco detection topic used by the panel tracker.",
            ),
            DeclareLaunchArgument(
                "panel_tracker_ema_alpha",
                default_value="0.2",
                description="EMA smoothing factor for panel pose tracking.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
