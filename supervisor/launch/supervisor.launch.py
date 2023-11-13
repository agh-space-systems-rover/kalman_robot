from ament_index_python import get_package_share_path
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
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
                        / "realsense.yaml"
                    ),
                }.items(),
            )
            
        ]
    )
