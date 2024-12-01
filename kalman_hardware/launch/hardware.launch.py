import os
from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration

REALSENSE_SERIAL_NUMBERS = {
    "d455_front": "_043422251512",
    "d455_back": "_231622302908",
    "d455_left": "_231622302763",
    "d455_right": "_231122300896",
    "d435_arm": "_936322070029",
}
PHIDGETS_CONTAINER_NAME = "phidgets_container"


def launch_setup(context):
    def get_bool(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"

    def get_str(name):
        return LaunchConfiguration(name).perform(context)
    
    def get_float(name):
        return float(LaunchConfiguration(name).perform(context))

    component_container = get_str("component_container")
    master = get_str("master")
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]
    imu = get_bool("imu")
    compass_calibration = get_float("compass_calibration")
    gps = get_bool("gps")

    if len(rgbd_ids) > 0:
        rgbd_ids_sns = [
            (x, REALSENSE_SERIAL_NUMBERS[x])
            for x in rgbd_ids
            if x in REALSENSE_SERIAL_NUMBERS
        ]

    if imu and compass_calibration > 1e-6:
        raise RuntimeError(
            "IMU cannot be started simultaneously with the compass_calibration node. Please start either one of them separately or do not start them at all."
        )

    description = []

    if master != "":
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_master")
                        / "launch"
                        / "master.launch.py"
                    )
                ),
                launch_arguments={
                    "mode": master,
                }.items(),
            )
        ]

    # RGBD cameras are togglable.
    if rgbd_ids:
        # Those nodes facilitate the communication with the RealSense devices
        # and publish data to ROS topics.
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("realsense2_camera")
                        / "launch"
                        / "rs_launch.py"
                    )
                ),
                launch_arguments={
                    "camera_name": camera_name,
                    "camera_namespace": "",
                    "serial_no": serial_no,
                    "config_file": str(  # Must use external config file for non-configurable options.
                        get_package_share_path("kalman_hardware")
                        / "config"
                        / "realsense2_camera.yaml"
                    ),
                }.items(),
            )
            for camera_name, serial_no in rgbd_ids_sns
        ]
    # TODO: Composition

    # The IMU may also be disabled to allow for compass calibration.
    if imu:
        # Get the path to the calibration parameters file.
        phidgets_spatial_calibration_params_path = os.path.abspath(
            os.path.join(
                os.path.expanduser("~"),
                ".config/kalman/phidgets_spatial_calibration_params.yaml",
            )
        )

        # Throw if the calibration parameters file does not exist.
        if not os.path.exists(phidgets_spatial_calibration_params_path):
            raise RuntimeError(
                "Cannot launch without calibration parameters. Please start the compass_calibration node and invoke the calibration service to generate the required configuration file."
            )

        # Load IMU driver and filter.
        if component_container:
            description += [
                LoadComposableNodes(
                    target_container=component_container,
                    composable_node_descriptions=[
                        ComposableNode(
                            package="phidgets_spatial",
                            plugin="phidgets::SpatialRosI",
                            parameters=[
                                str(
                                    get_package_share_path("kalman_hardware")
                                    / "config"
                                    / "phidgets_spatial.yaml"
                                ),
                                phidgets_spatial_calibration_params_path,
                            ],
                            # NOTE: Spatial does not support intra-process communication.
                        ),
                        ComposableNode(
                            package="imu_filter_madgwick",
                            plugin="ImuFilterMadgwickRos",
                            parameters=[
                                str(
                                    get_package_share_path("kalman_hardware")
                                    / "config"
                                    / "imu_filter_madgwick.yaml"
                                ),
                            ],
                            extra_arguments=[{"use_intra_process_comms": True}],
                        ),
                    ],
                ),
            ]
        else:
            description += [
                ComposableNodeContainer(
                    package="rclcpp_components",
                    executable="component_container",
                    namespace="",
                    name=PHIDGETS_CONTAINER_NAME,
                    composable_node_descriptions=[
                        ComposableNode(
                            package="phidgets_spatial",
                            plugin="phidgets::SpatialRosI",
                            parameters=[
                                str(
                                    get_package_share_path("kalman_hardware")
                                    / "config"
                                    / "phidgets_spatial.yaml"
                                ),
                                phidgets_spatial_calibration_params_path,
                            ],
                        ),
                    ],
                ),
                Node(
                    package="imu_filter_madgwick",
                    executable="imu_filter_madgwick_node",
                    parameters=[
                        str(
                            get_package_share_path("kalman_hardware")
                            / "config"
                            / "imu_filter_madgwick.yaml"
                        )
                    ],
                ),
            ]

    if compass_calibration > 1e-6:
        description += [
            Node(
                package="kalman_hardware",
                executable="compass_calibration",
                parameters=[
                    {
                        "duration": compass_calibration,
                    }
                ],
            ),
        ]

    if gps:
        description += [
            Node(
                package="nmea_navsat_driver",
                executable="nmea_serial_driver",
                parameters=[
                    str(
                        get_package_share_path("kalman_hardware")
                        / "config"
                        / "nmea_navsat_driver.yaml"
                    )
                ],
                remappings=[
                    ("fix", "gps/fix"),
                    ("heading", "gps/heading"),
                    ("vel", "gps/vel"),
                    ("time_reference", "gps/time_reference"),
                ],
                respawn=True,
            ),
        ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="",
                description="Name of an existing component container to use. Empty by default to disable composition.",
            ),
            DeclareLaunchArgument(
                "master", default_value="", description="Start the master driver in a given mode ('pc', 'gs' or 'arm'). Leave empty to disable."
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                default_value="",
                description="Space-separated IDs of the depth cameras to use (d455_front, d455_back, ...). Leave empty to disable the cameras.",
            ),
            DeclareLaunchArgument(
                "imu", default_value="false", description="Start the IMU driver."
            ),
            DeclareLaunchArgument(
                "compass_calibration",
                default_value="0.0",
                description="Start IMU compass calibration node for a given number of seconds. IMU must be disabled in order to calibrate the compass. Zero to run in normal mode, without calibration.",
            ),
            DeclareLaunchArgument(
                "gps", default_value="false", description="Start the GPS driver."
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
