from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context):
    component_container = (
        LaunchConfiguration("component_container").perform(context).strip()
    )
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]

    description = [
        Node(
            package="kalman_arch",
            executable="cube_saver",
            parameters=[
                {
                    "buffer_size": 100,
                    "buffer_cleanup_time": 5000,
                    "num_cameras": len(rgbd_ids),
                }
            ],
            remappings=[
                (
                    f"annotated{i}/image_raw/compressed",
                    f"{camera_id}/yolo_annotated/compressed",
                )
                for i, camera_id in enumerate(rgbd_ids)
            ],
        ),
    ]

    record_video_params = {
        "frame_rate": 15,
    }

    if component_container:
        description += [
            LoadComposableNodes(
                target_container=component_container,
                composable_node_descriptions=[
                    ComposableNode(
                        package="kalman_arch",
                        plugin="kalman_arch::RecordVideo",
                        namespace=camera_id,
                        parameters=[
                            {
                                "camera_name": camera_id,
                                "image_transport": "raw",
                            },
                            record_video_params,
                        ],
                        remappings=[
                            (
                                "image_raw",
                                "color/image_raw",
                            )
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    )
                    for camera_id in rgbd_ids
                ],
            )
        ]
    else:
        for camera_id in rgbd_ids:
            description += [
                Node(
                    package="kalman_arch",
                    executable="record_video",
                    namespace=camera_id,
                    parameters=[
                        {
                            "camera_name": camera_id,
                            "image_transport": "compressed",
                        },
                        record_video_params,
                    ],
                    remappings=[
                        (
                            "image_raw",
                            "color/image_raw",
                        )
                    ],
                )
            ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="",
                description="Name of an existing component container to use. Empty to disable composition.",
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
