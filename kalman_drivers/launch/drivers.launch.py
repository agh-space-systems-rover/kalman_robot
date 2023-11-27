import os

from ament_index_python import get_package_share_path

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration

REALSENSE_SERIAL_NUMBERS = {
    "d455_front": "_043422251512",
    "d455_back": "_231622302908",
    # TODO: left cam
    "d455_right": "_231122300896",
}


def launch_setup(context):
    realsense_ids = [
        x
        for x in LaunchConfiguration("realsense_ids").perform(context).split(" ")
        if x != ""
    ]
    realsense_ids_sns = [
        (x, REALSENSE_SERIAL_NUMBERS[x])
        for x in realsense_ids
        if x in REALSENSE_SERIAL_NUMBERS
    ]

    description = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    get_package_share_path("kalman_master_driver")
                    / "launch"
                    / "master_com.launch.py"
                )
            )
        ),
        Node(
            package="kalman_master_driver",
            executable="wheel_driver",
        ),
        Node(
            package="kalman_master_driver",
            executable="ueuos_driver",
        ),
        Node(
            package="kalman_drivers",
            executable="compasscal",
        ),
        ComposableNodeContainer(
            name="phidget_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="phidgets_spatial",
                    plugin="phidgets::SpatialRosI",
                    name="phidgets_spatial",
                    parameters=[
                        str(
                            get_package_share_path("kalman_drivers")
                            / "param"
                            / "phidgets_spatial.yaml"
                        ),
                        os.path.abspath(
                            os.path.join(
                                os.path.expanduser("~"),
                                ".config/kalman/phidgets_spatial_calibration_params.yaml",
                            )
                        ),
                    ],
                ),
            ],
        ),
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            parameters=[
                str(
                    get_package_share_path("kalman_drivers")
                    / "param"
                    / "imu_filter_madgwick.yaml"
                )
            ],
        ),
    ]

    if len(realsense_ids_sns) > 0:
        description += [
            # ---------------------------
            # real-life RealSense drivers
            # ---------------------------
            # Those nodes facilitate the communication with the RealSense devices
            # and publish data to ROS topics.
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
                    "serial_no": serial_no,
                    "config_file": str(  # Must use external config file for non-configurable options.
                        get_package_share_path("kalman_drivers")
                        / "param"
                        / "realsense2_camera.yaml"
                    ),
                }.items(),
            )
            for camera_name, serial_no in realsense_ids_sns
            # TODO: Add the fourth camera here.
        ]
        description += [
            # Compressed re-publishers
            Node(
                package="image_transport",
                executable="republish",
                arguments=["raw", "compressed"],
                remappings=[
                    ("in", f"/{camera_name}/color/image_raw"),
                    ("out/compressed", f"/{camera_name}/color/image_raw/compressed"),
                ],
            )
            for camera_name, serial_no in realsense_ids_sns
        ]
        # TODO: Use compressed_depth_image_transport for depth images?

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "realsense_ids",
                default_value="d455_front d455_back d455_back d455_right",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
