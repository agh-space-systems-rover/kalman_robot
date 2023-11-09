from ament_index_python import get_package_share_path
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

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
            ) for camera_name, serial_no in [("d455_front", "_043422251512")]
        ] + [
            Node(
                package="image_transport",
                executable="republish",
                arguments=["raw", "compressed"],
                remappings=[
                    ("in", f"/{camera_name}/color/image_raw"),
                    ("out/compressed", f"/{camera_name}/color/image_raw/compressed"),
                ],
            ) for camera_name in ["d455_front"]
        ]
    )
