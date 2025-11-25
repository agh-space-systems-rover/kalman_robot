from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration

from kalman_utils.launch import launch_node_or_load_component, load_standalone_config


def launch_point_cloud_utils_nodes(
    name: str,
    parameters: list,
    remappings: list,
    component_container: str,
    rgbd_ids: list,
) -> list:
    plugin = (
        "point_cloud_utils::" + "".join([x.capitalize() for x in name.split("_")])
        if component_container
        else None
    )

    actions = []
    for camera_id in rgbd_ids:
        actions += launch_node_or_load_component(
            component_container=component_container,
            package="point_cloud_utils",
            executable=name,
            plugin=plugin,
            namespace=camera_id,
            parameters=parameters,
            remappings=remappings,
        )
    return actions


def launch_setup(context):
    component_container = (
        LaunchConfiguration("component_container").perform(context).strip()
    )
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]

    actions = []

    # cloud generation
    parameters = [
        # str(get_package_share_path("kalman_clouds") / "config" / "rgbd_cloud.yaml"),
        load_standalone_config("kalman_clouds", "rgbd_cloud.yaml"),
        (
            {
                "color_transport": "compressed",
                "depth_transport": "compressedDepth",
            }
            if not component_container
            else {}
        ),
    ]
    remappings = [
        ("color/image_raw", "color/image_raw"),
        ("depth/image_raw", "depth/image_raw"),
        ("cloud", "point_cloud/raw"),
    ]
    actions += launch_point_cloud_utils_nodes(
        "rgbd_cloud", parameters, remappings, component_container, rgbd_ids
    )

    # voxel grid
    parameters = [
        load_standalone_config("kalman_clouds", "voxel_grid.yaml"),
    ]
    remappings = [
        ("input", "point_cloud/raw"),
        ("output", "point_cloud/grid"),
    ]
    actions += launch_point_cloud_utils_nodes(
        "voxel_grid", parameters, remappings, component_container, rgbd_ids
    )

    # outlier removal
    parameters = [
        load_standalone_config("kalman_clouds", "radius_outlier_removal.yaml"),
    ]
    remappings = [
        ("input", "point_cloud/grid"),
        ("output", "point_cloud"),
    ]
    actions += launch_point_cloud_utils_nodes(
        "radius_outlier_removal", parameters, remappings, component_container, rgbd_ids
    )

    return actions


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
                description="Space-separated IDs of the depth cameras to use for point cloud generation.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
