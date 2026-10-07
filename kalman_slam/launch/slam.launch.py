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

from kalman_utils.launch import launch_node_or_load_component, load_standalone_config


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
    component_container = LaunchConfiguration("component_container").perform(context)
    gps_datum = [
        float(x)
        for x in LaunchConfiguration("gps_datum").perform(context).split(" ")
        if x != ""
    ]
    lio_config = LaunchConfiguration("lio_config").perform(context)
    use_mag = LaunchConfiguration("use_mag").perform(context).lower() == "true"
    use_rtabmap = LaunchConfiguration("use_rtabmap").perform(context).lower() == "true"
    fiducials = LaunchConfiguration("fiducials").perform(context)

    description = []

    # Crop the cloud
    description += launch_node_or_load_component(
        component_container=component_container,
        package="pcl_ros",
        executable="crop_box",
        plugin="pcl_ros::CropBox",
        name="crop_box_node",
        parameters=[
            {
                "min_x": -0.3,
                "max_x": 1.1,
                "min_y": -0.5,
                "max_y": 0.5,
                "min_z": -0.3,
                "max_z": 0.8,
                "negative": True,
            }
        ],
        remappings=[
            ("input", "/livox/points"),
            ("output", "/livox/points/cropped"),
        ],
    )

    # Setup LIO
    description += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(get_package_share_path("fast_lio") / "launch" / "mapping.launch.py")
            ),
            launch_arguments={
                "config_file": str(
                    get_package_share_path("kalman_slam")
                    / "config"
                    / f"{lio_config}.yaml"
                ),
                "rviz": "false",
            }.items(),
        ),
        # FAST-LIO estimates the raw built-in IMU pose in camera_init. Attach
        # the upside-down sensor frames to the ROS odom/base_link tree.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="fast_lio_odom_to_camera_init",
            arguments=[
                "--x",
                "-0.439",
                "--y",
                "-0.02329",
                "--z",
                "0.47412",
                "--qx",
                "1.0",
                "--qy",
                "0.0",
                "--qz",
                "0.0",
                "--qw",
                "0.0",
                "--frame-id",
                "odom",
                "--child-frame-id",
                "camera_init",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="fast_lio_body_to_base_link",
            arguments=[
                "--x",
                "0.439",
                "--y",
                "-0.02329",
                "--z",
                "0.47412",
                "--qx",
                "1.0",
                "--qy",
                "0.0",
                "--qz",
                "0.0",
                "--qw",
                "0.0",
                "--frame-id",
                "body",
                "--child-frame-id",
                "base_link",
            ],
        ),
    ]

    # Convert FAST-LIO's raw camera_init -> body odometry
    # odom -> base_link representation consumed by localization and navigation.
    # This is done becouse the livox lidar is mounted upside down.
    # FAST-LIO2 does not provide any parameters to set this transformation :(
    description += [
        Node(
            package="kalman_slam",
            executable="odometry_frame_transformer",
            name="fast_lio_odometry_frame_transformer",
            remappings=[
                ("odometry/raw", "/Odometry"),
                ("odometry/transformed", "/odometry/local"),
            ],
        ),
    ]

    # Setup RTAB-Map
    # This is only used when navigating without GPS.
    # Added to compensate odom -> map drift
    # Needs fine-tuning and real-world verification
    if use_rtabmap:
        description += [
            Node(
                namespace="rtabmap",
                name="slam",
                executable="rtabmap",
                package="rtabmap_slam",
                parameters=[
                    str(
                        get_package_share_path("kalman_slam")
                        / "config"
                        / ("rtabmap_slam.yaml")
                    )
                ],
                remappings=[
                    ("odom", "/odometry/local"),
                    ("scan_cloud", "/cloud_registered_body"),
                ],
                arguments=["-d", "--ros-args", "--log-level", "warn"],
            ),
            # For development:
            # Node(
            #     namespace="rtabmap",
            #     executable="rtabmap_viz",
            #     package="rtabmap_viz",
            #     remappings={
            #         "odom": "/odometry/local",
            #     }.items(),
            # ),
        ]

    # Setup EKF and global odometry
    # FAST-LIO publishes odom -> camera_init -> body -> base_link
    description += [
        # Global Kalman filter, publishes map->base_link.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_global",
            parameters=[
                load_ekf_config(
                    "ekf_filter_node_global",
                    use_mag=use_mag,
                    use_rtabmap=use_rtabmap,
                )
            ],
            remappings=[("odometry/filtered", "odometry/global")],
        ),
        # navsat_transform_node will refuse to work on messages with NaN values, so a custom preprocessor is needed.
        # This preprocessor will also set initial covariance to 0 to guarantee fast EKF convergence.
        Node(
            package="kalman_slam",
            executable="gps_preprocessor",
            remappings=[
                ("fix", "gps/fix"),
                ("fix/filtered", "gps/fix/filtered"),
            ],
        ),
        # This node creates a service that can be used to send a fake GPS fix.
        # This service is forwarded over RF to be used on the ground station.
        # Why not send the GPS fix directly from the ground station?
        # For now topics are not ACKed by ros_link (RF comms),
        # so forwarding them instead of a service would be less reliable.
        Node(
            package="kalman_slam",
            executable="gps_spoofer",
            remappings=[
                ("spoof_gps", "spoof_gps"),
                ("spoof_gps/look_at", "spoof_gps/look_at"),
                ("imu", "imu/spoofed"),
                ("fix/in", "gps/filtered"),
                ("fix/out", "gps/fix"),
            ],
        ),
        # Navsat transform listens to global odometry (map->base_link) from the second Kalman filter, ekf_filter_node_gps.
        # The node listens for GPS fixes from the sensor (or gps_spoofer) and will use them to correct the global odometry for drift.
        # The corrected global odometry will then be republished on a different topic and used by ekf_filter_node_global
        # to apply the correction to global odometry.
        # Additionally, the global odometry is converted to a GPS fix relative to a starting position (the datum).
        # We use this fix to display the robot's position on the map when no GPS sensor is available.
        # TODO: Might need to delay this node a bit so that global EKF converges before navsat_transform initialization.
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            parameters=[
                str(
                    get_package_share_path("kalman_slam")
                    / "config"
                    / "navsat_transform_node.yaml"
                ),
                (
                    {
                        # Datum is the starting position of the robot.
                        # Those GPS coordinates correspond to the location of the "map" frame in the real world.
                        "datum": [gps_datum[0], gps_datum[1], 0.0],
                        "wait_for_datum": True,  # Needs to be set to enable the datum.
                    }
                    if gps_datum
                    else {
                        "wait_for_datum": False,
                    }
                ),
            ],
            remappings={
                # IMU is technically not needed here because navsat_transform_node has the heading from global odometry which is world-referenced because it uses absolute yaw readings from the IMU.
                # Global odometry can be used instead of the IMU sensor by setting a "use_odometry_yaw" parameter,
                # but this did not work reliably for some reason.
                "imu": "imu/data",  # the node subscribes to imu, not imu/data.
                # Those are the GPS sensor readings (filtered by gps_preprocessor).
                "gps/fix": "gps/fix/filtered",
                # This is the current global odometry state republished as a GPS fix.
                # "gps/filtered": "gps/filtered",
                # And this is the correctd global odometry as a nav_msgs/Odometry message.
                # "odometry/gps": "odometry/gps",
                # This is the current global odometry from ekf_filter_node_gps.
                # It has approximately the same value as the corrected global odometry,
                # but keep in mind that a separate topic is needed for the correction mechanism to work.
                "odometry/filtered": "odometry/global",
            }.items(),
        ),
    ]

    # if fiducials != "":
    #     description += [
    #         Node(
    #             package="kalman_slam",
    #             executable="fiducial_odometry",
    #             parameters=[
    #                 str(
    #                     get_package_share_path("kalman_slam")
    #                     / "config"
    #                     / "fiducial_odometry.yaml"
    #                 ),
    #                 {
    #                     "fiducials_path": str(
    #                         get_package_share_path("kalman_slam")
    #                         / "fiducials"
    #                         / f"{fiducials}.yaml"
    #                     ),
    #                 },
    #             ],
    #             remappings=[("odometry", "odometry/fiducial")],
    #         ),
    #     ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lio_config",
                default_value="mid360s",
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
                "component_container",
                default_value="",
                description="Name of an existing component container to use. Empty to disable composition.",
            ),
            DeclareLaunchArgument(
                "use_rtabmap",
                default_value="false",
                choices=["true", "false"],
                description="Enable if loop closure detection is needed. Only when navigating without GPS!",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
