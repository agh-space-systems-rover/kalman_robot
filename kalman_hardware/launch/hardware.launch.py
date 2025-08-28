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

from kalman_utils.launch import (
    launch_node_or_load_component,
    load_standalone_config,
    load_standalone_config_file,
)

REALSENSE_SERIAL_NUMBERS = {
    #"d455_front": "_242422301926",
    "d455_front": "_043422251512",
    "d455_front_old": "_043422251512",
    "d455_back": "_241122302098",
    "d455_left": "_231622302763",
    "d455_right": "_231122300896",
    "d435_arm": "_936322070029",
    "d455_back_old": "_231622302908",
}
PHIDGETS_CONTAINER_NAME = "phidgets_container"
REALSENSE_CONTAINER_NAME = "realsense_container"


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
    imu = get_str("imu")
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

    actions = []

    if master != "":
        actions += [
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
        composable_node_descriptions = sum(
            [
                [
                    ComposableNode(
                        namespace=camera_name,
                        name="raw",
                        package="realsense2_camera",
                        plugin="realsense2_camera::RealSenseNodeFactory",
                        parameters=[
                            {
                                "serial_no": serial_no,
                                "camera_name": camera_name,
                            },
                            load_standalone_config(
                                "kalman_hardware", "realsense2_camera.yaml"
                            ),
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                    ComposableNode(
                        namespace=camera_name,
                        name=f"filter",
                        package="kalman_hardware",
                        plugin="kalman_hardware::RgbdFilter",
                        parameters=[
                            load_standalone_config(
                                "kalman_hardware", "rgbd_filter.yaml"
                            )
                        ],
                        remappings=[
                            ("in/color/image_raw", "raw/color/image_raw"),
                            (
                                "in/depth/image_raw",
                                "raw/aligned_depth_to_color/image_raw",
                            ),
                            ("in/color/camera_info", "raw/color/camera_info"),
                            ("out/color/image_raw", "color/image_raw"),
                            ("out/depth/image_raw", "depth/image_raw"),
                            ("out/color/camera_info", "color/camera_info"),
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ]
                for camera_name, serial_no in rgbd_ids_sns
            ],
            [],
        )
        if component_container:
            actions += [
                LoadComposableNodes(
                    target_container=component_container,
                    composable_node_descriptions=composable_node_descriptions,
                )
            ]
        else:
            actions += [
                ComposableNodeContainer(
                    package="rclcpp_components",
                    executable="component_container",
                    namespace="",
                    name=REALSENSE_CONTAINER_NAME,
                    composable_node_descriptions=composable_node_descriptions,
                ),
            ]

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
                "Cannot launch without calibration parameters. Please launch kalman_hardware with compass_calibration set to desired calibration duration and drive the rover around. For autonomous calibration, use:\nros2 launch kalman_bringup util_compasscal.launch.py"
            )

        # Load IMU driver and filter.
        actions += launch_node_or_load_component(
            component_container=component_container,
            package="phidgets_spatial",
            executable="phidgets_spatial_node",
            plugin="phidgets::SpatialRosI",
            parameters=[
                load_standalone_config("kalman_hardware", "phidgets_spatial.yaml"),
                load_standalone_config_file(phidgets_spatial_calibration_params_path),
            ],
            extra_arguments=[{"use_intra_process_comms": False}],
        )
        # ^ NOTE: Spatial does not support intra-process communication.

        actions += launch_node_or_load_component(
            component_container=component_container,
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            plugin="ImuFilterMadgwickRos",
            parameters=[
                load_standalone_config("kalman_hardware", "imu_filter_madgwick.yaml"),
                {"use_mag": imu != "no_mag"},
            ],
        )

    if compass_calibration > 1e-6:
        actions += [
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
        actions += [
            Node(
                package="nmea_navsat_driver",
                executable="nmea_serial_driver",
                parameters=[
                    load_standalone_config("kalman_hardware", "nmea_navsat_driver.yaml")
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

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="",
                description="Name of an existing component container to use. Empty by default to disable composition.",
            ),
            DeclareLaunchArgument(
                "master",
                default_value="",
                choices=["", "pc", "gs", "arm"],
                description="Start the master driver in a given mode ('pc', 'gs' or 'arm'). Leave empty to disable.",
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                default_value="",
                description="Space-separated IDs of the depth cameras to use (d455_front, d455_back, ...). Leave empty to disable the cameras.",
            ),
            DeclareLaunchArgument(
                "imu",
                default_value="",
                choices=["", "no_mag", "full"],
                description="Start IMU. 'no_mag' disables magnetometer.",
            ),
            DeclareLaunchArgument(
                "compass_calibration",
                default_value="0.0",
                description="Start IMU compass calibration node for a given number of seconds. IMU must be disabled in order to calibrate the compass. Zero to run in normal mode, without calibration.",
            ),
            DeclareLaunchArgument(
                "gps",
                default_value="false",
                choices=["true", "false"],
                description="Start the GPS driver.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
