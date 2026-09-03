import yaml
from ament_index_python.packages import get_package_share_path
from kalman_utils.launch import launch_node_or_load_component, remap_action
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)
    servo_config_path = (
        get_package_share_path("kalman_arm") / "config" / "servo_config.yaml"
    )
    with servo_config_path.open() as servo_config_file:
        servo_config = yaml.safe_load(servo_config_file)
    joint_limit_params = {
        "joint_limit_margins": servo_config["joint_limit_margins"]
    }

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
        "min_linear_speed": 0.0,
        "fine_approach_angular_scale": 0.4,
        "max_linear_speed": 0.4,
        "position_tolerance": 0.005,
        "visual_refinement_position_tolerance": 0.005,
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

    actions += launch_node_or_load_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="pose_ik",
        plugin="kalman_arm2::PoseIK",
        namespace="arm",
        remappings=[
            ("current_pos", "current_pos"),
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
        parameters=[joint_limit_params],
    )

    panel_layout_file = PathJoinSubstitution(
        [FindPackageShare("kalman_arm2"), "config", "panel_layout.yaml"]
    )


    panel_tracker_params = {
        "layout_yaml": ParameterValue(panel_layout_file, value_type=str),
        "tracking_frame": LaunchConfiguration("panel_tracking_frame").perform(context),
        "board_frame": LaunchConfiguration("panel_board_frame").perform(context),
        "detection_topic": LaunchConfiguration("panel_detection_topic").perform(
            context
        ),
        "depth_topic": LaunchConfiguration("panel_depth_topic").perform(context),
        "camera_info_topic": LaunchConfiguration(
            "panel_camera_info_topic"
        ).perform(context),
        "ema_alpha": float(
            LaunchConfiguration("panel_tracker_ema_alpha").perform(context)
        ),
        "depth_refinement_enabled": yaml.safe_load(
            LaunchConfiguration("panel_depth_refinement_enabled").perform(context)
        ),
        "depth_max_age_s": float(
            LaunchConfiguration("panel_depth_max_age_s").perform(context)
        ),
        "plane_roi_margin_m": float(
            LaunchConfiguration("panel_plane_roi_margin_m").perform(context)
        ),
        "plane_initial_distance_m": float(
            LaunchConfiguration("panel_plane_initial_distance_m").perform(context)
        ),
        "plane_min_residual_threshold_m": float(
            LaunchConfiguration("panel_plane_min_residual_threshold_m").perform(context)
        ),
        "plane_robust_sigma_multiplier": float(
            LaunchConfiguration("panel_plane_robust_sigma_multiplier").perform(context)
        ),
        "plane_iterations": int(
            LaunchConfiguration("panel_plane_iterations").perform(context)
        ),
        "plane_max_normal_update_deg": float(
            LaunchConfiguration("panel_plane_max_normal_update_deg").perform(context)
        ),
        "plane_max_offset_update_m": float(
            LaunchConfiguration("panel_plane_max_offset_update_m").perform(context)
        ),
        "plane_min_points": int(
            LaunchConfiguration("panel_plane_min_points").perform(context)
        ),
        "plane_pixel_stride": int(
            LaunchConfiguration("panel_plane_pixel_stride").perform(context)
        ),
    }
    panel_rectifier_params = {
        "layout_yaml": ParameterValue(panel_layout_file, value_type=str),
        "board_frame": LaunchConfiguration("panel_board_frame").perform(context),
        "image_topic": LaunchConfiguration("panel_image_topic").perform(context),
        "depth_topic": LaunchConfiguration("panel_depth_topic").perform(context),
        "camera_info_topic": LaunchConfiguration(
            "panel_camera_info_topic"
        ).perform(context),
        "sync_queue_size": int(
            LaunchConfiguration("panel_sync_queue_size").perform(context)
        ),
        "sync_tolerance_s": float(
            LaunchConfiguration("panel_sync_tolerance_s").perform(context)
        ),
        "pixels_per_meter": float(
            LaunchConfiguration("panel_pixels_per_meter").perform(context)
        ),
        "left_border_m": float(
            LaunchConfiguration("panel_left_border_m").perform(context)
        ),
        "top_border_m": float(
            LaunchConfiguration("panel_top_border_m").perform(context)
        ),
        "draw_board_outline": yaml.safe_load(
            LaunchConfiguration("panel_draw_board_outline").perform(context)
        ),
    }

    visual_refinement_params = {
        "visual_refinement_dof": int(
            LaunchConfiguration("visual_refinement_dof").perform(context)
        ),
        "visual_refinement_panel_frame": LaunchConfiguration(
            "panel_board_frame",
        ).perform(context),
        # "visual_refinement_twist_angular_kp": 0.4,
        # "visual_refinement_twist_max_angular_speed": 0.1,
        # "visual_refinement_max_measurement_age_s": 0.1,
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
                visual_refinement_params,
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

    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("kalman_aruco"), "launch", "aruco.launch.py"]
                )
            ),
            launch_arguments={
                "component_container": component_container,
                "rgbd_ids": "d455_arm_wheel",
                "dict": "ARUCO_ORIGINAL",
                "size": "0.050",
            }.items(),
        )
    ]

    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("kalman_yolo"), "launch", "yolo.launch.py"]
                )
            ),
            launch_arguments={
                "rgbd_ids": "d455_arm_wheel",
                "config": "panel",
            }.items(),
        )
    ]

    actions += launch_node_or_load_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="panel_tracker",
        plugin="kalman_arm2::PanelTracker",
        namespace="arm",
        parameters=[panel_tracker_params],
    )

    actions += launch_node_or_load_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="panel_rectifier",
        plugin="kalman_arm2::PanelRectifier",
        namespace="arm",
        parameters=[panel_rectifier_params],
    )

    actions += launch_node_or_load_component(
        component_container=component_container,
        package="kalman_arm2",
        executable="panel_click_marker",
        plugin="kalman_arm2::PanelClickMarker",
        namespace="arm",
        parameters=[
            {
                "board_frame": LaunchConfiguration("panel_board_frame").perform(
                    context
                )
            }
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
        ("/joy_compressed", "kalman_interfaces/msg/ArmCompressed", "send"),
    ]:
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
            parameters=[joint_limit_params],
        )
    ]

    actions += [
        Node(
            package="kalman_arm2",
            executable="rock_detector",
            namespace="arm",
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
                default_value="0.02",
                description="Base damping factor for twist IK.",
            ),
            DeclareLaunchArgument(
                "ik_max_damping",
                default_value="0.08",
                description="Maximum damping factor for twist IK near singularities.",
            ),
            DeclareLaunchArgument(
                "ik_singularity_sigma_threshold",
                default_value="0.12",
                description="Smallest singular value threshold below which damping ramps up.",
            ),
            DeclareLaunchArgument(
                "ik_joint_centering_gain",
                default_value="1.5",
                #default_value="0.0",
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
                default_value="[1.0, 1.0, 1.0, 5.0, 0.3, 0.3]",
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
                default_value="/d455_arm_wheel/aruco_detections",
                description="Aruco detection topic used by the panel tracker.",
            ),
            DeclareLaunchArgument(
                "panel_tracker_ema_alpha",
                default_value="0.2",
                description="EMA smoothing factor for panel pose tracking.",
            ),
            DeclareLaunchArgument(
                "panel_depth_refinement_enabled",
                default_value="true",
                description="Refine marker panel pose with aligned depth plane fitting.",
            ),
            DeclareLaunchArgument(
                "panel_depth_max_age_s",
                default_value="0.1",
                description="Maximum depth-to-ArUco timestamp difference.",
            ),
            DeclareLaunchArgument(
                "panel_plane_roi_margin_m",
                default_value="0.02",
                description="Inward panel-bound margin used for plane candidates.",
            ),
            DeclareLaunchArgument(
                "panel_plane_initial_distance_m",
                default_value="0.02",
                description="Initial depth slab around marker-estimated plane.",
            ),
            DeclareLaunchArgument(
                "panel_plane_min_residual_threshold_m",
                default_value="0.002",
                description="Minimum MAD trimming threshold for plane inliers.",
            ),
            DeclareLaunchArgument(
                "panel_plane_robust_sigma_multiplier",
                default_value="3.0",
                description="MAD multiplier used during iterative plane trimming.",
            ),
            DeclareLaunchArgument(
                "panel_plane_iterations",
                default_value="3",
                description="Number of iterative plane fit and trim passes.",
            ),
            DeclareLaunchArgument(
                "panel_plane_max_normal_update_deg",
                default_value="5.0",
                description="Maximum accepted depth correction of panel normal.",
            ),
            DeclareLaunchArgument(
                "panel_plane_max_offset_update_m",
                default_value="0.01",
                description="Maximum accepted depth correction along panel normal.",
            ),
            DeclareLaunchArgument(
                "panel_plane_min_points",
                default_value="200",
                description="Minimum depth points required for panel-plane fitting.",
            ),
            DeclareLaunchArgument(
                "panel_plane_pixel_stride",
                default_value="2",
                description="Depth pixel subsampling stride used by plane fitting.",
            ),
            DeclareLaunchArgument(
                "panel_image_topic",
                default_value="/d455_arm_wheel/color/image_raw",
                description="Raw panel camera image consumed by the rectifier.",
            ),
            DeclareLaunchArgument(
                "panel_depth_topic",
                default_value="/d455_arm_wheel/depth/image_raw",
                description="Aligned depth image consumed by the orthographic renderer.",
            ),
            DeclareLaunchArgument(
                "panel_camera_info_topic",
                default_value="/d455_arm_wheel/color/camera_info",
                description="Panel camera calibration consumed by the rectifier.",
            ),
            DeclareLaunchArgument(
                "panel_sync_queue_size",
                default_value="30",
                description="Queue size for approximate panel RGB-D synchronization.",
            ),
            DeclareLaunchArgument(
                "panel_sync_tolerance_s",
                default_value="0.02",
                description="Maximum timestamp skew for panel RGB-D synchronization.",
            ),
            DeclareLaunchArgument(
                "panel_pixels_per_meter",
                default_value="1000.0",
                description="Scale of the top-down panel image.",
            ),
            DeclareLaunchArgument(
                "panel_left_border_m",
                default_value="0.15",
                description="Extra view outside the YAML panel boundary on its left.",
            ),
            DeclareLaunchArgument(
                "panel_top_border_m",
                default_value="0.05",
                description="Extra view outside the YAML panel boundary above it.",
            ),
            DeclareLaunchArgument(
                "panel_draw_board_outline",
                default_value="true",
                description="Draw a green outline around the YAML panel area.",
            ),
            DeclareLaunchArgument(
                "visual_refinement_dof",
                default_value="6",
                choices=["3", "6"],
                description="Visual final correction mode: translation only (3) or full pose (6).",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
