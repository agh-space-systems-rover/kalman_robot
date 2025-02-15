from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_path

REALSENSE_CAMERAS = ["d455_front", "d455_left", "d455_right", "d455_back"]


def launch_setup(context):
    description = []
    for camera in REALSENSE_CAMERAS:
        description += [
            Node(
                package="kalman_cubes",
                executable="detections",
                parameters=[
                    {
                        "camera_no": camera,
                    }
                ]
            )
        ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_no",
                default_value="",
                description="Each name of the realsense cameras"
            ),
            OpaqueFunction(function=launch_setup)
        ]
    )
