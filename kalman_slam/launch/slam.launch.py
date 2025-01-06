from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
import jinja2
import yaml
import os


def load_ekf_config(name, **kwargs) -> str:
    # Load EKF config template
    with open(
        str(get_package_share_path("kalman_slam") / "config" / f"{name}.yaml.j2"),
        "r",
    ) as f:
        ekf_config_template = jinja2.Template(f.read())

    # Render EKF config with camera IDs
    ekf_config = ekf_config_template.render(**kwargs)

    # Load EKF config
    ekf_params = yaml.load(ekf_config, Loader=yaml.FullLoader)

    # Save EKF config to file
    ekf_params_path = f"/tmp/kalman/{name}." + str(os.getpid()) + ".yaml"
    os.makedirs(os.path.dirname(ekf_params_path), exist_ok=True)
    with open(ekf_params_path, "w") as f:
        yaml.dump(ekf_params, f)

    # Return path to EKF config
    return ekf_params_path


def find_available_fiducial_configs() -> set[str]:
    fiducials_dir = get_package_share_path("kalman_slam") / "fiducials"
    configs = [f.stem for f in fiducials_dir.glob("*.yaml")]
    return set(configs)


def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]
    gps_datum = [
        float(x)
        for x in LaunchConfiguration("gps_datum").perform(context).split(" ")
        if x != ""
    ]
    fiducials = LaunchConfiguration("fiducials").perform(context)
    use_mag = LaunchConfiguration("use_mag").perform(context).lower() == "true"

    description = []

    # visual odometry
    if component_container:
        if rgbd_ids:
            description += [
                LoadComposableNodes(
                    target_container=component_container,
                    composable_node_descriptions=[
                        ComposableNode(
                            package="rtabmap_odom",
                            plugin="rtabmap_odom::RGBDOdometry",
                            namespace=f"{camera_id}",
                            parameters=[
                                str(
                                    get_package_share_path("kalman_slam")
                                    / "config"
                                    / "rgbd_odometry.yaml"
                                )
                            ],
                            remappings={
                                "rgb/image": f"color/image_raw",
                                "depth/image": f"aligned_depth_to_color/image_raw",
                                "rgb/camera_info": f"color/camera_info",
                            }.items(),
                            extra_arguments=[{"use_intra_process_comms": True}],
                        )
                        for camera_id in rgbd_ids
                    ],
                ),
            ]
    else:
        description += [
            Node(
                namespace=f"{camera_id}",
                package="rtabmap_odom",
                executable="rgbd_odometry",
                parameters=[
                    str(
                        get_package_share_path("kalman_slam")
                        / "config"
                        / "rgbd_odometry.yaml"
                    )
                ],
                remappings=[
                    ("rgb/image", f"color/image_raw"),
                    ("depth/image", f"aligned_depth_to_color/image_raw"),
                    ("rgb/camera_info", f"color/camera_info"),
                ],
                arguments=["--ros-args", "--log-level", "error"],
            )
            for camera_id in rgbd_ids
        ]

    description += [
        # Local Kalman filter, publishes odom->base_link.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_local",
            parameters=[load_ekf_config("ekf_filter_node_local", camera_ids=rgbd_ids)],
            remappings=[("odometry/filtered", "odometry/local")],
        ),
        # Global Kalman filter, publishes map->base_link.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_global",
            parameters=[load_ekf_config("ekf_filter_node_global", use_mag=use_mag)],
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
                ("fix", "gps/fix"),
            ],
        ),
        # Navsat transform listens to global odometry (map->base_link) from the second Kalman filter, ekf_filter_node_gps.
        # The node listens for GPS fixes from the sensor (or gps_spoofer) and will use them to correct the global odometry for drift.
        # The corrected global odometry will then be republished on a different topic and used by ekf_filter_node_gps
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

    if fiducials != "":
        # Fiducial odometry
        description += [
            Node(
                package="kalman_slam",
                executable="fiducial_odometry",
                parameters=[
                    str(
                        get_package_share_path("kalman_slam")
                        / "config"
                        / "fiducial_odometry.yaml"
                    ),
                    {
                        "fiducials_path": str(
                            get_package_share_path("kalman_slam")
                            / "fiducials"
                            / f"{fiducials}.yaml"
                        ),
                    },
                ],
                remappings=[("odometry", "odometry/fiducial")],
            ),
        ]

    # if mapping:
    #     raise NotImplementedError("Mapping is not implemented yet.")
    # sync
    # parameters = [
    #     str(get_package_share_path("kalman_clouds") / "config" / "cloud_sync.yaml"),
    #     {
    #         "number_of_inputs": len(rgbd_ids),
    #     },
    # ]
    # remappings = [("output", "point_cloud/raw")] + (
    #     [
    #         (f"input{i}", f"{camera_id}/point_cloud")
    #         for i, camera_id in enumerate(rgbd_ids)
    #     ]
    #     if len(rgbd_ids) > 1
    #     else [("input", f"{rgbd_ids[0]}/point_cloud")] if len(rgbd_ids) == 1 else []
    # )
    # description += [
    #     launch_point_cloud_utils_node("cloud_sync", parameters, remappings, component_container)
    # ]
    # description += [
    #     Node(
    #         namespace=f"rtabmap",
    #         package="rtabmap_slam",
    #         executable="rtabmap",
    #         parameters=[
    #             str(
    #                 get_package_share_path("kalman_slam") / "config" / "rtabmap.yaml"
    #             )
    #         ],
    #         remappings={
    #             "scan_cloud": "point_cloud_sync/points",
    #             "odom": "odometry/filtered",
    #         }.items(),
    #         arguments=["--delete_db_on_start"],
    #     ),
    # ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="",
                description="Name of an existing component container to use. Empty to disable composition.",
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                default_value="",
                description="Space-separated IDs of the depth cameras to use.",
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
            OpaqueFunction(function=launch_setup),
        ]
    )
