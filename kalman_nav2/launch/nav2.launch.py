from ament_index_python import get_package_share_path
from launch_ros.actions import Node
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
import copy

from kalman_utils.launch import launch_node_or_load_component, load_standalone_config


NAV_CONTAINER_NAME = "nav2_container"


def recursive_dict_update(old, new):
    for k, v in new.items():
        if isinstance(v, dict):
            old[k] = recursive_dict_update(old.get(k, {}), v)
        else:
            old[k] = copy.deepcopy(v)
    return old


def render_jinja_config(template_path, **kwargs):
    # Load template
    with open(template_path) as f:
        template = jinja2.Template(f.read())

    # Render config
    config_str = template.render(**kwargs)

    # Load and return the rendered config
    config = yaml.load(config_str, Loader=yaml.FullLoader)
    return config


def find_available_maps() -> set[str]:
    maps_dir = get_package_share_path("kalman_nav2") / "maps"
    maps = [f.stem for f in maps_dir.glob("*.yaml")]
    return set(maps)


def render_nav2_config(rgbd_ids, static_map):
    # Render core Nav2 params.
    nav2_params = render_jinja_config(
        str(get_package_share_path("kalman_nav2") / "config" / "nav2.yaml.j2"),
        rgbd_ids=rgbd_ids,
        static_map=static_map,
    )

    # Render commons costmap params.
    costmap_params = render_jinja_config(
        str(get_package_share_path("kalman_nav2") / "config" / "nav2.costmap.yaml.j2"),
        rgbd_ids=rgbd_ids,
        static_map=static_map,
    )

    # Overlay Nav2 local_costmap params onto common costmap params.
    local_costmap_params = nav2_params["local_costmap"]["local_costmap"][
        "ros__parameters"
    ]
    recursive_dict_update(local_costmap_params, costmap_params)
    # Set local_costmap params in core Nav2 config to the merged params
    nav2_params["local_costmap"]["local_costmap"][
        "ros__parameters"
    ] = local_costmap_params

    # Same for global costmap.
    global_costmap_params = nav2_params["global_costmap"]["global_costmap"][
        "ros__parameters"
    ]
    recursive_dict_update(global_costmap_params, costmap_params)
    nav2_params["global_costmap"]["global_costmap"][
        "ros__parameters"
    ] = global_costmap_params

    # Save final Nav2 config to a file.
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
    driving_mode = LaunchConfiguration("driving_mode").perform(context)

    actions = []

    # obstacle detection
    actions += sum(
        (
            launch_node_or_load_component(
                component_container=component_container,
                package="point_cloud_utils",
                executable="obstacle_detection",
                plugin="point_cloud_utils::ObstacleDetection",
                namespace=camera_id,
                parameters=[
                    load_standalone_config("kalman_nav2", "obstacle_detection.yaml"),
                ],
                remappings=[
                    ("input", "point_cloud"),
                    ("output", "point_cloud/obstacles"),
                ],
                extra_arguments=[{"use_intra_process_comms": False}],
            ) for camera_id in rgbd_ids
        ),
        [],
    )

    # static map
    if static_map:
        actions += [
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
    if component_container:
        # We need to spawn a separate container for Nav2.
        # It is not possible to load Nav2 parameters into an
        # existing container without having access to its Node definition.
        actions += [
            Node(
                package="rclcpp_components",
                executable="component_container_mt",
                name=NAV_CONTAINER_NAME,
                parameters=[nav2_params_path],  # Required due to bugs in rclcpp.
                # See: https://github.com/ros-planning/navigation2/issues/4011
            ),
        ]
    actions += [
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
                "use_composition": "True" if component_container else "False",
                "container_name": NAV_CONTAINER_NAME,
            }.items(),
        ),
    ]

    # path follower
    # This is a Python module and it does not support composition.
    actions += [
        Node(
            package="kalman_nav2",
            executable="path_follower",
            parameters=[
                str(
                    get_package_share_path("kalman_nav2")
                    / "config"
                    / "path_follower.yaml"
                ),
                {
                    "driving_mode": driving_mode,
                },
            ],
        ),
    ]

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
                default_value="",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            DeclareLaunchArgument(
                "static_map",
                default_value="",
                choices=["", *find_available_maps()],
                description="Name of the static map to use. Maps are stored in kalman_nav2/maps. Empty by default to disable static map.",
            ),
            DeclareLaunchArgument(
                "driving_mode",
                default_value="hybrid",
                choices=["hybrid", "forward", "backward"],
                description="Direction to drive in. The default 'hybrid' mode allows driving in both directions.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
