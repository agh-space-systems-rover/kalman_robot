from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode

def launch_point_cloud_utils_nodes(name: str, parameters: list, remappings: list, component_container: str, rgb_ids: list) -> list:
    if component_container:
        plugin = "point_cloud_utils::" + "".join([x.capitalize() for x in name.split('_')])
        return [LoadComposableNodes(
            target_container=component_container,
            composable_node_descriptions=[
                ComposableNode(
                    package="point_cloud_utils",
                    plugin=plugin,
                    namespace=camera_id,
                    parameters=parameters,
                    remappings=remappings,
                    extra_arguments=[{"use_intra_process_comms": True}],
                ) for camera_id in rgb_ids
            ],
        )]
    else:
        return [Node(
            package="point_cloud_utils",
            executable=name,
            namespace=camera_id,
            parameters=parameters,
            remappings=remappings,
        ) for camera_id in rgb_ids]


def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context).strip()
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]

    description = []

    # cloud generation
    parameters = [
        str(get_package_share_path("kalman_clouds") / "config" / "rgbd_cloud.yaml")
    ]
    remappings = [
        ("color/image_raw", "color/image_raw"),
        ("depth/image_raw", "aligned_depth_to_color/image_raw"),
        ("cloud", "point_cloud/raw"),
    ]
    description += launch_point_cloud_utils_nodes("rgbd_cloud", parameters, remappings, component_container, rgbd_ids)

    # voxel grid
    parameters = [
        str(get_package_share_path("kalman_clouds") / "config" / "voxel_grid.yaml")
    ]
    remappings = [
        ("input", "point_cloud/raw"),
        ("output", "point_cloud/grid"),
    ]
    description += launch_point_cloud_utils_nodes("voxel_grid", parameters, remappings, component_container, rgbd_ids)

    # outlier removal
    parameters = [
        str(get_package_share_path("kalman_clouds") / "config" / "radius_outlier_removal.yaml")
    ]
    remappings = [
        ("input", "point_cloud/grid"),
        ("output", "point_cloud"),
    ]
    description += launch_point_cloud_utils_nodes("radius_outlier_removal", parameters, remappings, component_container, rgbd_ids)
    # parameters = [
    #     str(get_package_share_path("kalman_clouds") / "config" / "statistical_outlier_removal.yaml")
    # ]
    # remappings = [
    #     ("input", "point_cloud/grid"),
    #     ("output", "point_cloud"),
    # ]
    # description += launch_point_cloud_utils_nodes("statistical_outlier_removal", parameters, remappings, component_container, rgbd_ids)

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                description="Name of an existing component container to use. Empty to disable composition.",
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                description="Space-separated IDs of the depth cameras to use for point cloud generation.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
