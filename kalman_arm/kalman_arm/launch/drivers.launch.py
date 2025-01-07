import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_path


def generate_launch_description():

    gps_node = Node(
        package="nmea_navsat_driver",
        executable="nmea_serial_driver",
        parameters=[
            str(
                get_package_share_path("kalman_arm")
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
        respawn_delay=30,
    )

    phidgets_spatial_calibration_params_path = os.path.abspath(
        os.path.join(
            os.path.expanduser("~"),
            ".config/kalman/phidgets_spatial_calibration_params.yaml",
        )
    )

    imu_container = ComposableNodeContainer(
        name="imu_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        respawn=True,
        composable_node_descriptions=[
            ComposableNode(
                package="phidgets_spatial",
                plugin="phidgets::SpatialRosI",
                parameters=[
                    str(
                        get_package_share_path("kalman_arm")
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
        output="both",
    )

    return LaunchDescription(
        [
            gps_node,
            imu_container,
        ]
    )
