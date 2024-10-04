from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode


def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]
    dict = LaunchConfiguration("dict").perform(context)
    size = float(LaunchConfiguration("size").perform(context))

    description = []

    parameters = [
        str(get_package_share_path("kalman_aruco") / "config" / f"aruco_tracker.yaml"),
        {
            "marker_dict": dict,
            "marker_size": size,
        },
    ]

    if component_container:
        description += [
            LoadComposableNodes(
                target_container=component_container,
                composable_node_descriptions=[
                    ComposableNode(
                        package="aruco_opencv",
                        plugin="aruco_opencv::ArucoTrackerAutostart",
                        namespace=rgbd_id,
                        name="aruco_tracker",
                        parameters=parameters,
                        extra_arguments=[{"use_intra_process_comms": True}],
                    )
                    for rgbd_id in rgbd_ids
                ],
            )
        ]
    else:
        description += [
            Node(
                package="aruco_opencv",
                executable="aruco_tracker_autostart",
                namespace=rgbd_id,
                name="aruco_tracker",
                parameters=parameters,
            )
            for rgbd_id in rgbd_ids
        ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="",
                description="Name of an existing component container to use. Empty by default to disable composition.",
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                default_value="",
                description="Space-separated IDs of the RGBD cameras to use. Regular cameras are not supported yet.",
            ),
            DeclareLaunchArgument(
                "dict",
                default_value="4X4_50",
                description="Dictionary of markers to use.",
            ),
            DeclareLaunchArgument(
                "size",
                default_value="0.15",
                description="Size of the marker including black border in meters.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
