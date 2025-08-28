from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from kalman_utils.launch import launch_node_or_load_component, load_standalone_config


def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]
    dict = LaunchConfiguration("dict").perform(context)
    size = float(LaunchConfiguration("size").perform(context))

    actions = []

    parameters = [
        load_standalone_config("kalman_aruco", "aruco_tracker.yaml"),
        {
            "marker_dict": dict,
            "marker_size": size,
        },
    ]

    actions += sum((launch_node_or_load_component(
        component_container=component_container,
        package="aruco_opencv",
        executable="aruco_tracker_autostart",
        plugin="aruco_opencv::ArucoTrackerAutostart",
        namespace=rgbd_id,
        name="aruco_tracker",
        parameters=parameters,
    ) for rgbd_id in rgbd_ids), [])

    return actions


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
                description="Space-separated IDs of the RGBD cameras to use. Regular cameras are not supported yet.",
            ),
            DeclareLaunchArgument(
                "dict",
                default_value="4X4_50",
                choices=[
                    f"{x}X{x}_{s}"
                    for x in [4, 5, 6, 7, 8]
                    for s in [50, 100, 250, 1000]
                ]
                + ["ARUCO_ORIGINAL"],
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
