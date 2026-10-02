from gc import enable
from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)

from launch.substitutions import LaunchConfiguration
import jinja2
from rclpy import parameter
import yaml
import os


def load_ekf_config(name, **kwargs) -> str:
    with open(
        str(get_package_share_path("kalman_slam") / "config" / f"{name}.yaml.j2"),
        "r",
    ) as f:
        ekf_config_template = jinja2.Template(f.read())

    ekf_config = ekf_config_template.render(**kwargs)
    ekf_params = yaml.load(ekf_config, Loader=yaml.FullLoader)

    ekf_params_path = f"/tmp/kalman/{name}." + str(os.getpid()) + ".yaml"
    os.makedirs(os.path.dirname(ekf_params_path), exist_ok=True)
    with open(ekf_params_path, "w") as f:
        yaml.dump(ekf_params, f)

    return ekf_params_path


def find_available_fiducial_configs() -> set[str]:
    fiducials_dir = get_package_share_path("kalman_slam") / "fiducials"
    configs = [f.stem for f in fiducials_dir.glob("*.yaml")]
    return set(configs)


def launch_setup(context):
    lio_config = LaunchConfiguration("lio_config").perform(context)
    gps_datum = [
        float(x)
        for x in LaunchConfiguration("gps_datum").perform(context).split(" ")
        if x != ""
    ]
    fiducials = LaunchConfiguration("fiducials").perform(context)
    use_mag = LaunchConfiguration("use_mag").perform(context).lower() == "true"
    enable_loop_closures = (
        LaunchConfiguration("enable_loop_closures").perform(context).lower() == "true"
    )

    description = []

    # Setup LIO
    description += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(get_package_share_path("fast_lio") / "launch" / "mapping.launch.py")
            ),
            launch_arguments={
                "config_file": str(
                    get_package_share_path("kalman_slam") / "config" / f"{lio_config}.yaml"
                ),
                "rviz": "true",
            }.items(),
        ),
        # FAST-LIO estimates the raw built-in IMU pose in camera_init. Attach
        # the upside-down sensor frames to the ROS odom/base_link tree.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="fast_lio_odom_to_camera_init",
            arguments=[
                "--x", "-0.439", "--y", "-0.02329", "--z", "0.47412",
                "--qx", "1.0",
                "--qy", "0.0",
                "--qz", "0.0",
                "--qw", "0.0",
                "--frame-id", "odom",
                "--child-frame-id", "camera_init",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="fast_lio_body_to_base_link",
            arguments=[
                "--x", "0.439",
                "--y", "-0.02329",
                "--z", "0.47412",
                "--qx", "1.0",
                "--qy", "0.0",
                "--qz", "0.0",
                "--qw", "0.0",
                "--frame-id", "body",
                "--child-frame-id", "base_link",
            ],
        ),
    ]
    # Setup KISS-Matcher loop closure detection (only for navigation without GNSS)
    if enable_loop_closures:
        description += [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    str(
                        get_package_share_path("kiss_matcher_ros")
                        / "launch"
                        / "run_kiss_matcher_sam.launch.yaml"
                    )
                ),
                launch_arguments={
                    "odom_topic": "/Odometry",
                    "scan_topic": "/cloud_registered_body",
                    "start_rviz": "true",
                    "map_frame": "map",
                    "base_frame": "base_link",
                }.items(),
            )
        ]
    

    # # Setup EKF and global odometry
    # description += [
    #     Node(
    #         package="robot_localization",
    #         executable="ekf_node",
    #         name="ekf_filter_node_local",
    #         parameters=[load_ekf_config("ekf_filter_node_local")],
    #         remappings=[("odometry/filtered", "odometry/local")],
    #     ),
    #     Node(
    #         package="robot_localization",
    #         executable="ekf_node",
    #         name="ekf_filter_node_global",
    #         parameters=[load_ekf_config("ekf_filter_node_global", use_mag=use_mag)],
    #         remappings=[("odometry/filtered", "odometry/global")],
    #     ),
    #     Node(
    #         package="kalman_slam",
    #         executable="gps_preprocessor",
    #         remappings=[
    #             ("fix", "gps/fix"),
    #             ("fix/filtered", "gps/fix/filtered"),
    #         ],
    #     ),
    #     Node(
    #         package="kalman_slam",
    #         executable="gps_spoofer",
    #         remappings=[
    #             ("spoof_gps", "spoof_gps"),
    #             ("spoof_gps/look_at", "spoof_gps/look_at"),
    #             ("imu", "imu/spoofed"),
    #             ("fix/in", "gps/filtered"),
    #             ("fix/out", "gps/fix"),
    #         ],
    #     ),
    #     Node(
    #         package="robot_localization",
    #         executable="navsat_transform_node",
    #         parameters=[
    #             str(
    #                 get_package_share_path("kalman_slam")
    #                 / "config"
    #                 / "navsat_transform_node.yaml"
    #             ),
    #             (
    #                 {
    #                     "datum": [gps_datum[0], gps_datum[1], 0.0],
    #                     "wait_for_datum": True,
    #                 }
    #                 if gps_datum
    #                 else {
    #                     "wait_for_datum": False,
    #                 }
    #             ),
    #         ],
    #         remappings={
    #             "imu": "imu/data",  # Phidget IMU topic
    #             "gps/fix": "gps/fix/filtered",
    #             "odometry/filtered": "odometry/global",
    #         }.items(),
    #     ),
    # ]

    # # if fiducials != "":
    # #     description += [
    # #         Node(
    # #             package="kalman_slam",
    # #             executable="fiducial_odometry",
    # #             parameters=[
    # #                 str(
    # #                     get_package_share_path("kalman_slam")
    # #                     / "config"
    # #                     / "fiducial_odometry.yaml"
    # #                 ),
    # #                 {
    # #                     "fiducials_path": str(
    # #                         get_package_share_path("kalman_slam")
    # #                         / "fiducials"
    # #                         / f"{fiducials}.yaml"
    # #                     ),
    # #                 },
    # #             ],
    # #             remappings=[("odometry", "odometry/fiducial")],
    # #         ),
    # #     ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lio_config",
                default_value="mid360s.yaml",
                description="FAST-LIO configuration file from kalman_slam/config.",
            ),
            DeclareLaunchArgument(
                "gps_datum",
                default_value="",
                description="The 'latitude longitude' of the map frame. Only used if GPS is enabled. Empty to assume first recorded GPS fix.",
            ),
            DeclareLaunchArgument(
                "fiducials",
                default_value="",
                choices=["", *find_available_fiducial_configs()],
                description="Name of the list of fiducials to use. Empty disables fiducial odometry.",
            ),
            DeclareLaunchArgument(
                "use_mag",
                default_value="true",
                choices=["true", "false"],
                description="Use IMU yaw readings for global EKF. If disabled, heading will drift over time.",
            ),
            DeclareLaunchArgument(
                "enable_loop_closures",
                default_value="false",
                choices=["true", "false"],
                description="Enable loop closure detection, needed when navigating without GNSS",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
