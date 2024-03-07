from ament_index_python import get_package_share_path
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
import yaml
import os
import jinja2


def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]
    obstacle_detection = LaunchConfiguration("obstacle_detection").perform(context).lower() == "true"

    # Init a container if not provided.
    description = []
    if component_container == "":
        description += [
            Node(
                package="rclcpp_components",
                executable="component_container_mt",
                name="kalman_nav2_container",
            )
        ]
        component_container = "kalman_nav2_container"

    # Obstacle detection
    if obstacle_detection:
        description += [
            # Node(
            #     package="point_cloud_utils",
            #     executable="obstacle_detection",
            #     parameters=[
            #         str(
            #             get_package_share_path("kalman_nav2")
            #             / "param"
            #             / "obstacle_detection.yaml"
            #         ),
            #     ],
            #     remappings={
            #         "input": f"/{camera_id}/depth/color/points/filtered",
            #         "output": f"/{camera_id}/depth/color/points/filtered/obstacles",
            #     }.items(),
            # )
            # for camera_id in rgbd_ids
            LoadComposableNodes(
                target_container=component_container,
                composable_node_descriptions=[
                    ComposableNode(
                        package="point_cloud_utils",
                        plugin="point_cloud_utils::ObstacleDetection",
                        name=f"{camera_id}_obstacle_detection",
                        remappings={
                            "input": f"/{camera_id}/depth/color/points/filtered",
                            "output": f"/{camera_id}/depth/color/points/filtered/obstacles",
                        }.items(),
                    )
                    for camera_id in rgbd_ids
                ]
            ),
        ]

    # Load Nav2 config template
    with open(
        str(get_package_share_path("kalman_nav2") / "param" / "nav2.yaml.j2"),
    ) as f:
        nav2_config_template = jinja2.Template(f.read())

    # Render config using camera IDs
    ukf_config_str = nav2_config_template.render(
        rgbd_ids=rgbd_ids,
        obstacle_detection=obstacle_detection,
    )

    # Load config
    nav2_params = yaml.load(ukf_config_str, Loader=yaml.FullLoader)

    # Save it to file
    nav2_params_path = "/tmp/kalman/nav2." + str(os.getpid()) + ".yaml"
    os.makedirs(os.path.dirname(nav2_params_path), exist_ok=True)
    with open(nav2_params_path, "w") as f:
        yaml.dump(nav2_params, f)

    description += [
        # NOTE: It is not possible to load Nav2 into an existing container without having access to its Node definition.
        Node(
            package="rclcpp_components",
            executable="component_container",
            name="kalman_nav2_container_nav",
            parameters=[nav2_params_path],
            # Required due to bugs in rclcpp.
            # See: https://github.com/ros-planning/navigation2/issues/4011
        ),
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
                "use_composition": "True",
                "container_name": "kalman_nav2_container_nav",
            }.items(),
        ),
        # path follower
        Node(
            package="kalman_nav2",
            executable="path_follower",
            parameters=[
                str(
                    get_package_share_path("kalman_nav2")
                    / "param"
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
                "component_container", default_value="", description="Name of an existing component container to use."
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                default_value="d455_front d455_back d455_left d455_right",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            DeclareLaunchArgument(
                "obstacle_detection",
                default_value="true",
                description="Whether to use obstacle detection.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
