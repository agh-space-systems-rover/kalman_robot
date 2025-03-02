from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_path

REALSENSE_CAMERAS = ["d455_front", "d455_left", "d455_right", "d455_back"]


def launch_setup(context):
    description = []
    description += [
        Node(
            package="kalman_cubes",
            executable="detections",
            parameters=[
                {
                    "buffer_size": 100,
                    "buffer_cleanup_time": 5000,

                }
            ]
        ),
    ]

    for camera in REALSENSE_CAMERAS:
        description += [
            Node(
                package="kalman_cubes",
                executable="record_video",
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

            OpaqueFunction(function=launch_setup)
        ]
    )
