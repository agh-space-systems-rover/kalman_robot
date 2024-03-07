import os
from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration

REALSENSE_SERIAL_NUMBERS = {
    "d455_front": "_043422251512",
    "d455_back": "_231622302908",
    "d455_left": "_231622302763",
    "d455_right": "_231122300896",
}

def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)
    master = LaunchConfiguration("master").perform(context).lower() == "true"
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]
    imu = LaunchConfiguration("imu").perform(context).lower() == "true"
    compasscal = LaunchConfiguration("compasscal").perform(context).lower() == "true"

    if len(rgbd_ids) > 0:
        rgbd_ids_sns = [
            (x, REALSENSE_SERIAL_NUMBERS[x])
            for x in rgbd_ids
            if x in REALSENSE_SERIAL_NUMBERS
        ]

    if imu:
        if compasscal:
            raise RuntimeError(
                "IMU cannot be started simultaneously with the compasscal service node. Please start either one of them separately or do not start them at all."
            )

    description = []

    if master:
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_master_driver")
                        / "launch"
                        / "master_driver.launch.py"
                    )
                )
            )
        ]

    # RGBD cameras are togglable.
    if len(rgbd_ids) > 0:
        # Those nodes facilitate the communication with the RealSense devices
        # and publish data to ROS topics.
        if component_container:
            description += [
                LoadComposableNodes(
                    target_container=component_container,
                    composable_node_descriptions=[
                        ComposableNode(
                            package='realsense2_camera',
                            plugin='realsense2_camera::RealSenseNodeFactory',
                            name=camera_name,
                            parameters=[
                                {
                                    'camera_name': camera_name,
                                    'serial_no': serial_no,
                                },
                                str(
                                    get_package_share_path("kalman_drivers")
                                    / "config"
                                    / "realsense2_camera.yaml"
                                ),
                            ],
                            extra_arguments=[{"use_intra_process_comms": True}],
                        ) for camera_name, serial_no in rgbd_ids_sns
                    ],
                ),
            ]
        else:
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
                        "camera_name": f"{camera_name}",
                        "serial_no": serial_no,
                        "config_file": str(  # Must use external config file for non-configurable options.
                            get_package_share_path("kalman_drivers")
                            / "config"
                            / "realsense2_camera.yaml"
                        ),
                    }.items(),
                ) for camera_name, serial_no in rgbd_ids_sns
            ]

        # description += [
        #     # Compressed re-publishers
        #     Node(
        #         package="image_transport",
        #         executable="republish",
        #         arguments=["raw", "compressed"],
        #         remappings=[
        #             ("in", f"/{camera_name}/color/image_raw"),
        #             ("out/compressed", f"/{camera_name}/color/image_raw/compressed"),
        #         ],
        #     )
        #     for camera_name, serial_no in rgbd_ids_sns
        # ]
        # TODO: Use compressed_depth_image_transport for depth images?

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
                "Cannot launch without calibration parameters. Please start the compasscal node and invoke the calibration service to generate the required configuration file."
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
                                    get_package_share_path("kalman_drivers")
                                    / "config"
                                    / "phidgets_spatial.yaml"
                                ),
                                phidgets_spatial_calibration_params_path,
                            ],
                            extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                        ComposableNode(
                            package="imu_filter_madgwick",
                            plugin="ImuFilterMadgwickRos",
                            parameters=[
                                str(
                                    get_package_share_path("kalman_drivers")
                                    / "config"
                                    / "imu_filter_madgwick.yaml"
                                ),
                            ],
                            extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                    ],
                ),
            ]
        else:
            description += [
                Node(
                    package="phidgets_spatial",
                    executable="phidgets_spatial",
                    name="phidgets_spatial",
                    parameters=[
                        str(
                            get_package_share_path("kalman_drivers")
                            / "config"
                            / "phidgets_spatial.yaml"
                        ),
                        phidgets_spatial_calibration_params_path,
                    ],
                ),
                Node(
                    package="imu_filter_madgwick",
                    executable="imu_filter_madgwick_node",
                    parameters=[
                        str(
                            get_package_share_path("kalman_drivers")
                            / "config"
                            / "imu_filter_madgwick.yaml"
                        )
                    ],
                ),
            ]

    if compasscal:
        description += [
            # Compasscal is written in Python and thus is not composable.
            Node(
                package="kalman_drivers",
                executable="compasscal",
            ),
        ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="",
                description="Name of an existing component container to use. Empty by default to disable composition."
            ),
            DeclareLaunchArgument(
                "master", default_value="true", description="Start the master driver."
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                default_value="d455_front d455_back d455_left d455_right",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            DeclareLaunchArgument(
                "imu", default_value="true", description="Start the IMU driver."
            ),
            DeclareLaunchArgument(
                "compasscal",
                default_value="false",
                description="Start the IMU compass calibration service node. IMU must be disabled in order to calibrate the compass.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
