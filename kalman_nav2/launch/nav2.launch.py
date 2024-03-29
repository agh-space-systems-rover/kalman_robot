from ament_index_python import get_package_share_path
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
import yaml
import os
import jinja2

NAV_CONTAINER_NAME = "nav2_container"


def render_nav2_config(rgbd_ids, static_map):
    # Load Nav2 config template
    with open(
        str(get_package_share_path("kalman_nav2") / "config" / "nav2.yaml.j2"),
    ) as f:
        nav2_config_template = jinja2.Template(f.read())

    # Render config using camera IDs
    ukf_config_str = nav2_config_template.render(
        rgbd_ids=rgbd_ids,
        static_map=static_map,
    )

    # Load config
    nav2_params = yaml.load(ukf_config_str, Loader=yaml.FullLoader)

    # Save it to file
    nav2_params_path = "/tmp/kalman/nav2." + str(os.getpid()) + ".yaml"
    os.makedirs(os.path.dirname(nav2_params_path), exist_ok=True)
    with open(nav2_params_path, "w") as f:
        yaml.dump(nav2_params, f)

    return nav2_params_path


def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]
    static_map = LaunchConfiguration("static_map").perform(context)

    description = []

    # obstacle detection
    if len(rgbd_ids) > 0:
        if component_container:
            description += [
                LoadComposableNodes(
                    target_container=component_container,
                    composable_node_descriptions=[
                        ComposableNode(
                            package="point_cloud_utils",
                            plugin="point_cloud_utils::ObstacleDetection",
                            namespace=camera_id,
                            name="voxel_grid",
                            remappings={
                                "input": f"depth/color/points/filtered",
                                "output": f"depth/color/points/filtered/obstacles",
                            }.items(),
                        )
                        for camera_id in rgbd_ids
                    ],
                ),
            ]
        else:
            description += [
                Node(
                    package="point_cloud_utils",
                    executable="obstacle_detection",
                    parameters=[
                        str(
                            get_package_share_path("kalman_nav2")
                            / "config"
                            / "obstacle_detection.yaml"
                        ),
                    ],
                    remappings={
                        "input": f"/{camera_id}/depth/color/points/filtered",
                        "output": f"/{camera_id}/depth/color/points/filtered/obstacles",
                    }.items(),
                )
                for camera_id in rgbd_ids
            ]

    # static map
    if static_map:
        description += [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[
                    {
                        "yaml_filename": str(
                            get_package_share_path("kalman_nav2")
                            / "maps"
                            / f"{static_map}.yaml"
                        )
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {"use_sim_time": False},
                    {"autostart": True},
                    {"node_names": ["map_server"]},
                ],
            ),
        ]

    nav2_params_path = render_nav2_config(rgbd_ids, static_map)
    description += [
        # NOTE: It is not possible to load Nav2 parameters into an existing container without having access to its Node definition.
        # Node(
        #     package="rclcpp_components",
        #     executable="component_container_mt",
        #     name=NAV_CONTAINER_NAME,
        #     parameters=[nav2_params_path],  # Required due to bugs in rclcpp.
        #     # See: https://github.com/ros-planning/navigation2/issues/4011
        # ),
        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    get_package_share_path("kalman_nav2")
                    / "launch"
                    / "navigation_launch.launch.py"
                )
            ),
            launch_arguments={
                "params_file": nav2_params_path,
                "use_composition": "False",
                # "container_name": NAV_CONTAINER_NAME,
            }.items(),
        ),
    ]

    # path follower
    # This is a Python module and it does not support composition.
    description += [
        Node(
            package="kalman_nav2",
            executable="path_follower",
            parameters=[
                str(
                    get_package_share_path("kalman_nav2")
                    / "config"
                    / "path_follower.yaml"
                ),
            ],
        ),
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
                default_value="d455_front d455_back d455_left d455_right",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            DeclareLaunchArgument(
                "static_map",
                default_value="",
                description="Name of the static map to use. Maps are stored in kalman_nav2/maps. Empty by default to disable static map.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
