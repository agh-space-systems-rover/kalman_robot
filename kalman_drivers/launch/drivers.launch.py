import os

from ament_index_python import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription(
        # [
        #     # ---------------------------
        #     # real-life RealSense drivers
        #     # ---------------------------
        #     # Those nodes facilitate the communication with the RealSense devices
        #     # and publish data to ROS topics.
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(
        #             str(
        #                 get_package_share_path("realsense2_camera")
        #                 / "launch"
        #                 / "rs_launch.py"
        #             )
        #         ),
        #         launch_arguments={
        #             "camera_name": camera_name,
        #             "serial_no": serial_no,
        #             "config_file": str(  # Must use external config file for non-configurable options.
        #                 get_package_share_path("kalman_drivers")
        #                 / "param"
        #                 / "realsense2_camera.yaml"
        #             ),
        #         }.items(),
        #     ) for camera_name, serial_no in [("d455_front", "_043422251512"), ("d455_back", "_231622302908"), ("d455_right", "_231122300896")]
        #     # TODO: Add the fourth camera here.
        # ] + [
        #     # Compressed re-publishers
        #     Node(
        #         package="image_transport",
        #         executable="republish",
        #         arguments=["raw", "compressed"],
        #         remappings=[
        #             ("in", f"/{camera_name}/color/image_raw"),
        #             ("out/compressed", f"/{camera_name}/color/image_raw/compressed"),
        #         ],
        #     ) for camera_name in ["d455_front"]
        #     # TODO: Use compressed_depth_image_transport for depth images?
        # ]
        [
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
        ]
    )
