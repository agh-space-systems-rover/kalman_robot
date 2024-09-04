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


def load_local_ukf_config(rgbd_ids):
    # Load UKF config template
    with open(
        str(
            get_package_share_path("kalman_slam")
            / "config"
            / "ukf_filter_node_local.yaml.j2"
        ),
        "r",
    ) as f:
        ukf_config_template = jinja2.Template(f.read())

    # Render UKF config with camera IDs
    ukf_config = ukf_config_template.render(
        rgbd_ids=rgbd_ids,
    )

    # Load UKF config
    ukf_params = yaml.load(ukf_config, Loader=yaml.FullLoader)

    # Save UKF config to file
    ukf_params_path = "/tmp/kalman/ukf_filter_node_local." + str(os.getpid()) + ".yaml"
    os.makedirs(os.path.dirname(ukf_params_path), exist_ok=True)
    with open(ukf_params_path, "w") as f:
        yaml.dump(ukf_params, f)

    # Return path to UKF config
    return ukf_params_path


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

    description = []

    # visual odometry
    if component_container:
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

    # Local Kalman filter
    description += [
        Node(
            package="robot_localization",
            executable="ukf_node",
            name="ukf_filter_node",
            parameters=[load_local_ukf_config(rgbd_ids)],
            remappings=[("odometry/filtered", "odometry/local")],
        ),
    ]

    # GPS related nodes
    description += [
        # navsat_transform_node will refuse to work on messages with NaN values, so a custom preprocessor is needed.
        # This preprocessor will also set initial covariance to 0 to guarantee fast UKF convergence.
        # It also repairs any malformed spoof messages sent by groundstation over the RF link.
        Node(
            package="kalman_slam",
            executable="gps_preprocessor",
            remappings=[
                ("fix", "gps/fix"),
                ("fix/filtered", "gps/fix/filtered"),
            ],
        ),
        # This node creates a service that can be used to send a fake GPS fix.
        # In practice, this service is forwarded over RF to be used on the ground station.
        Node(
            package="kalman_slam",
            executable="gps_spoofer",
            remappings=[
                ("spoof_gps", "spoof_gps"),
                ("fix", "gps/fix"),
            ],
        ),
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
                        "wait_for_datum": True,
                        "datum": [gps_datum[0], gps_datum[1], 0.0],
                    }
                    if gps_datum
                    else {
                        "wait_for_datum": False,
                    }
                ),
            ],
            remappings={
                "imu": "imu/data",  # For some reason the node subscribes to imu, not imu/data.
                # IMU is theoretically not used by navsat_transform_node because it has the heading from filtered global odometry.
                "gps/fix": "gps/fix/filtered",
                # "gps/filtered": "gps/filtered",
                # "odometry/gps": "odometry/gps", # Sends raw GPS odometry to ukf_filter_node_gps.
                "odometry/filtered": "odometry/global",  # Receives filtered odometry from ukf_filter_node_gps.
            }.items(),
        ),
        Node(
            package="robot_localization",
            executable="ukf_node",
            name="ukf_filter_node_gps",
            parameters=[
                str(
                    get_package_share_path("kalman_slam")
                    / "config"
                    / "ukf_filter_node_global.yaml"
                )
                # In config:
                # - ukf_filter_node_gps receives raw GPS odometry from navsat_transform_node.
                # - ukf_filter_node_gps receives filtered IMU from sensors.
            ],
            remappings={
                "odometry/filtered": "odometry/global",  # Sends filtered odometry to navsat_transform_node.
            }.items(),
        ),
        # Origin pose at /local_xy_origin is needed for MapViz.
        # We use the UTM frame for navigation, but MapViz requires its own WGS84 origin.
        Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            parameters=[
                (
                    {
                        "local_xy_origin": "manual",  # != "auto"
                        "local_xy_origins": [
                            gps_datum[0],
                            gps_datum[1],
                            0.0,
                            0.0,
                        ],
                    }
                    if gps_datum
                    else {
                        "local_xy_origin": "auto",
                    }
                )
            ],
            remappings=[
                ("fix", "gps/fix/filtered"),
            ],
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
                remappings=[
                    ("odometry", "odometry/fiducial"),
                ],
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
                description="Name of an existing component container to use. Empty to disable composition.",
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            DeclareLaunchArgument(
                "gps_datum",
                description="The 'latitude longitude' of the map frame. Only used if GPS is enabled. Empty to assume first recorded GPS fix.",
            ),
            DeclareLaunchArgument(
                "fiducials",
                description="Name of the list of fiducials to use. Empty disables fiducial odometry.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
