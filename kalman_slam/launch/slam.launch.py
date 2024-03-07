from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.descriptions import ComposableNode

import jinja2
import yaml
import os


def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)
    localization = (
        LaunchConfiguration("localization").perform(context).lower() == "true"
    )
    mapping = LaunchConfiguration("mapping").perform(context).lower() == "true"
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]

    if mapping and not localization:
        raise RuntimeError(
            "Mapping cannot be enabled without localization. Please set localization=true."
        )
    
    # Init a container if not provided.
    description = []
    if component_container == "":
        description += [
            Node(
                package="rclcpp_components",
                executable="component_container_mt",
                name="kalman_slam_container",
            )
        ]
        component_container = "kalman_slam_container"

    # Spawn static transforms.
    description += [
        # static odom->base_link for testing
        Node(
            name="odom_to_base_link",
            package="tf2_ros",
            executable="static_transform_publisher",
            # arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            arguments=["--frame-id", "odom", "--child-frame-id", "base_link"],
            condition=UnlessCondition(LaunchConfiguration("localization")),
        ),
        # static map->odom transform
        Node(
            name="map_to_odom",
            package="tf2_ros",
            executable="static_transform_publisher",
            # arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        ),
    ]
    
    # Visual odometry
    description += [
        # Node(
        #     namespace=f"{camera_id}/rgbd_odometry",
        #     package="rtabmap_odom",
        #     executable="rgbd_odometry",
        #     parameters=[
        #         {
        #             "frame_id": "base_link",
        #             "publish_tf": False,
        #             # "subscribe_rgbd": True,
        #             # "rgbd_cameras": len(rgbd_ids),
        #             "approx_sync": False,
        #             # "approx_sync_max_interval": 0.01,
        #             "Odom/ResetCountdown": "1",
        #         }
        #     ],
        #     remappings=[
        #         ("rgb/image", f"/{camera_id}/color/image_raw"),
        #         ("depth/image", f"/{camera_id}/aligned_depth_to_color/image_raw"),
        #         ("rgb/camera_info", f"/{camera_id}/color/camera_info"),
        #     ],
        #     arguments=["--ros-args", "--log-level", "error"],
        # )
        # for camera_id in rgbd_ids
        LoadComposableNodes(
            target_container=component_container,
            composable_node_descriptions=[
                ComposableNode(
                    package="rtabmap_odom",
                    plugin="rtabmap_odom::RGBDOdometry",
                    namespace=f"{camera_id}/rgbd_odometry",
                    parameters=[
                        {
                            "frame_id": "base_link",
                            "publish_tf": False,
                            "approx_sync": False,
                            "Odom/ResetCountdown": "1",
                        }
                    ],
                    remappings={
                        "rgb/image": f"/{camera_id}/color/image_raw",
                        "depth/image": f"/{camera_id}/aligned_depth_to_color/image_raw",
                        "rgb/camera_info": f"/{camera_id}/color/camera_info",
                    }.items(),
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
                for camera_id in rgbd_ids
            ],
        ),
    ]

    # Load UKF config template
    with open(
        str(
            get_package_share_path("kalman_slam") / "param" / "ukf_filter_node.yaml.j2"
        ),
        "r",
    ) as f:
        ukf_config_template = jinja2.Template(f.read())
    # Render UKF config with camera IDs
    ukf_config = ukf_config_template.render(
        rgbd_ids=rgbd_ids,
    )
    # Load UKF config
    ukf_params = yaml.load(ukf_config, Loader=yaml.FullLoader)
    # Save UKF config to file
    ukf_params_path = "/tmp/kalman/ukf_filter_node." + str(os.getpid()) + ".yaml"
    os.makedirs(os.path.dirname(ukf_params_path), exist_ok=True)
    with open(ukf_params_path, "w") as f:
        yaml.dump(ukf_params, f)

    # Kalman filter
    description += [
        Node(
            package="robot_localization",
            executable="ukf_node",
            parameters=[ukf_params_path],
            condition=IfCondition(LaunchConfiguration("localization")),
        ),
    ]

    # point cloud clean-up
    description += [
        # Node(
        #     package='point_cloud_utils',
        #     executable='voxel_grid',
        #     parameters=[
        #         {
        #             "leaf_size": 0.1,
        #         }
        #     ],
        #     remappings=[
        #         ("input", f"/{camera_id}/depth/color/points"),
        #         ("output", f"/{camera_id}/depth/color/points/filtered"),
        #     ],
        # ) for camera_id in rgbd_ids
        LoadComposableNodes(
            target_container=component_container,
            composable_node_descriptions=[
                ComposableNode(
                    package="point_cloud_utils",
                    plugin="point_cloud_utils::VoxelGrid",
                    name=f"{camera_id}_voxel_grid",
                    remappings={
                        "input": f"/{camera_id}/depth/color/points",
                        "output": f"/{camera_id}/depth/color/points/filtered",
                    }.items(),
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
                for camera_id in rgbd_ids
            ],
        ),
    ]

    if mapping:
        description += [
            LoadComposableNodes(
                target_container=component_container,
                composable_node_descriptions=[
                    ComposableNode(
                        package="point_cloud_utils",
                        plugin="point_cloud_utils::PointCloudSync",
                        name="point_cloud_sync",
                        parameters=[
                            {
                                "number_of_inputs": len(rgbd_ids),
                            }
                        ],
                        remappings=[
                            (f"input{i}", f"/{camera_id}/depth/color/points/filtered")
                            for i, camera_id in enumerate(rgbd_ids)
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    )
                ],
            ),
            Node(
                namespace=f"rtabmap",
                package="rtabmap_slam",
                executable="rtabmap",
                parameters=[
                    str(
                        get_package_share_path("kalman_slam") / "param" / "rtabmap.yaml"
                    )
                ],
                remappings={
                    "scan_cloud": "/point_cloud_sync/points",
                    "odom": "/odometry/filtered",
                }.items(),
                arguments=["--delete_db_on_start"],
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
                "localization",
                default_value="true",
                description="Perform precise odom->base_link localization using the Unscented Kalman Filter.",
            ),
            DeclareLaunchArgument(
                "mapping",
                default_value="false",
                description="Create a 3D point cloud of the terrain as the robot moves. Localization must be enabled.",
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                default_value="d455_front d455_back d455_left d455_right",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
