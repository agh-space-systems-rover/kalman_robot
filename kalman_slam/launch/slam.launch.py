from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer

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

    description = [
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
        Node(
            namespace=f"{camera_id}/rgbd_odometry",
            package="rtabmap_odom",
            executable="rgbd_odometry",
            parameters=[
                {
                    "frame_id": "base_link",
                    "publish_tf": False,
                    # "subscribe_rgbd": True,
                    # "rgbd_cameras": len(rgbd_ids),
                    "approx_sync": False,
                    # "approx_sync_max_interval": 0.01,
                    "Odom/ResetCountdown": "1",
                }
            ],
            remappings=[
                ("rgb/image", f"/{camera_id}/color/image_raw"),
                ("depth/image", f"/{camera_id}/aligned_depth_to_color/image_raw"),
                ("rgb/camera_info", f"/{camera_id}/color/camera_info"),
            ],
            arguments=["--ros-args", "--log-level", "error"],
        )
        for camera_id in rgbd_ids
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
        Node(
            package='point_cloud_filters',
            executable='voxel_grid',
            parameters=[
                {
                    "leaf_size": 0.1,
                }
            ],
            remappings=[
                ("input", f"/{camera_id}/depth/color/points"),
                ("output", f"/{camera_id}/depth/color/points/filtered"),
            ],
        ) for camera_id in rgbd_ids
    ]

    if mapping:
        description += [
            Node(
                package="point_cloud_sync",
                executable="point_cloud_sync",
                parameters=[
                    {
                        "input_topics": [
                            f"/{camera_id}/depth/color/points/filtered" for camera_id in rgbd_ids
                        ],
                    }
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


# RTAB-Map SLAM
# ---
# ICP-only approach
# clouds do not align perfectly and generated normals are not great
# which leads to poor obstacle detection, but it is very fast
# ---
# Node(
#     package="point_cloud_sync",
#     executable="point_cloud_sync",
#     parameters=[
#         {
#             "input_topics": [
#                 "/d455_front/depth/color/points",
#                 "/d455_back/depth/color/points",
#                 "/d455_left/depth/color/points",
#                 "/d455_right/depth/color/points",
#             ],
#         }
#     ],
# ),
# Node(
#     namespace=f"rtabmap",
#     package="rtabmap_slam",
#     executable="rtabmap",
#     parameters=[
#         {
#             "subscribe_depth": False,
#             "subscribe_rgbd": False,
#             "subscribe_rgb": False,
#             "subscribe_stereo": False,
#             "subscribe_scan": False,
#             "subscribe_scan_cloud": True,
#             "subscribe_user_data": False,
#             "subscribe_odom_info": False,
#             "frame_id": "base_link",
#             "map_frame_id": "map",
#             "odom_frame_id": "odom",
#             "publish_tf": True,
#             "approx_sync": True,
#             "queue_size": 100,
#             "Rtabmap/DetectionRate": "1",  # update Hz
#             "Optimizer/Strategy": "2",  # GTSAM; default is ?
#             "Optimizer/Robust": "true",  # optimize paths using Vertigo; default is false
#             "Mem/NotLinkedNodesKept": "false",
#             "Mem/STMSize": "30",  # short-term memory size; default is 10
#             "Mem/LaserScanVoxelSize": "0.05",
#             "Reg/Strategy": "1",  # ICP; default is visodo=0
#             "Grid/CellSize": "0.05",  # resolution of occupancy grid; default is 0.05
#             "Grid/Sensor": "0",  # 0 = laser scan; 1 = RGB-D; 2 = both; 1 is default
#             "Grid/3D": "true",  # enables octomap; default is ?
#             # "Grid/NormalsSegmentation": "false",  # disabled for max ground height; default is true
#             # "Grid/MaxObstacleHeight": "1.5",  # max height of obstacles; default is disabled
#             # "Grid/ClusterRadius": "0.1",  # maximum radius of a cluster; default is 0.1
#             # "Grid/MinClusterSize": "30",  # minimum number of points in a cluster; default is 10
#             # "Grid/NormalK": "200",  # number of neighbors for normal estimation; default is 20
#             "Grid/MaxGroundAngle": "30",
#             "Grid/FlatObstacleDetected": "false",
#             # "Grid/RayTracing": "true",  # default is false
#             "Icp/VoxelSize": "0.05",  # disable uniform voxel size; default is 0.05
#             # "Icp/Iterations": "30",  # max iterations for ICP; default is 30
#             "OdomF2M/ScanSubtractRadius": "0.05",  # radius for filtering new points in local map; default is 0.05
#             "OdomF2M/ScanMaxSize": "10000",  # max number of points in local map; default is 2000
#         }
#     ],
#     remappings={
#         "scan_cloud": "/point_cloud_sync/points",
#         "odom": "/odometry/filtered",
#     }.items(),
#     arguments=["--delete_db_on_start"],
# ),
# ---
# RGB-D approach
# point clouds align pretty well and obstacle detection is good
# ---
# Node(
#     package="custom_rgbd_sync",
#     executable="custom_rgbd_sync",
#     parameters=[
#         {
#             "input_topics": [
#                 "/d455_front/rgbd_sync/rgbd_image",
#                 "/d455_back/rgbd_sync/rgbd_image",
#                 "/d455_left/rgbd_sync/rgbd_image",
#                 "/d455_right/rgbd_sync/rgbd_image",
#             ],
#         }
#     ],
# ),


# IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         str(
#             get_package_share_path("rtabmap_launch")
#             / "launch"
#             / "rtabmap.launch.py"
#         )
#     ),
#     launch_arguments={
#         "localization": "false",
#         "odom_topic": "/odometry/filtered",
#         "visual_odometry": "false",
#         "publish_tf_map": "false",
#         "publish_tf_odom": "false",
#         # "subscribe_rgbd": False,
#         "rgb_topic": "/d455_front/color/image_raw",
#         "depth_topic": "/d455_front/aligned_depth_to_color/image_raw",
#         "camera_info_topic": "/d455_front/color/camera_info",
#         "approx_sync": "true",
#         "rviz": "true",
#         "args": "--delete_db_on_start",
#     }.items(),
# ),


# if len(rgbd_ids) > 0:
#     description += [
#         Node(
#             namespace=f"rtabmap",
#             package="rtabmap_slam",
#             executable="rtabmap",
#             parameters=[
#                 {
#                     "subscribe_depth": False,
#                     "subscribe_rgb": False,
#                     "subscribe_rgbd": True,
#                     "subscribe_scan": False,
#                     "subscribe_scan_cloud": False,
#                     "subscribe_scan_descriptor": False,
#                     "subscribe_stereo": False,
#                     "subscribe_odom": True,
#                     "subscribe_odom_info": False,
#                     "subscribe_user_data": False,
#                     # "rgbd_cameras": 0,  # subscribe to rgbd_images, not rgbd_image
#                     # disabled because manual synchronization did not perform very well
#                     "rgbd_cameras": len(
#                         rgbd_ids
#                     ),  # subscribe to rgbd_image# topics if > 1, not rgbd_image
#                     "frame_id": "base_link",
#                     "map_frame_id": "map",
#                     "odom_frame_id": "odom",
#                     "publish_tf": True,
#                     "approx_sync": True,
#                     "queue_size": 50,
#                     "Rtabmap/DetectionRate": "1",  # update Hz
#                     "Optimizer/Strategy": "2",  # GTSAM; default is ?
#                     "Optimizer/Robust": "true",  # optimize paths using Vertigo; default is false
#                     "Mem/NotLinkedNodesKept": "false",
#                     "Mem/STMSize": "30",  # short-term memory size; default is 10
#                     "Mem/ImagePreDecimation": "4",  # default is 1 (divisor)
#                     # "Vis/Iterations": "300",  # default is 300
#                     "Grid/CellSize": "0.1",  # resolution of occupancy grid; default is 0.05
#                     "Grid/3D": "true",  # enables octomap; default is ?
#                     "Grid/MaxGroundAngle": "25",
#                     "Grid/FlatObstacleDetected": "false",
#                     # increase this if Grid/CelSize is decreased; it filters out small obstacles that are likely noise
#                     # "Grid/MinClusterSize": "30",  # minimum number of points in a cluster; default is 10
#                 }
#             ],
#             remappings=[("odom", "/odometry/filtered")]  # visual odometry
#             + (
#                 [
#                     # Multiple rgbd_image# topics if more than one realsense
#                     (
#                         f"rgbd_image{i}",
#                         f"/{rgbd_ids[i]}/rgbd_sync/rgbd_image",
#                     )
#                     for i in range(len(rgbd_ids))
#                 ]
#                 if len(rgbd_ids) > 1
#                 else [
#                     # Just a single topic otherwise
#                     ("rgbd_image", f"/{rgbd_ids[0]}/rgbd_sync/rgbd_image")
#                 ]
#             ),
#             arguments=["--delete_db_on_start"],
#             condition=IfCondition(LaunchConfiguration("mapping")),
#             # TODO: How to disable mapping but keep using RTAB-Map for obstacle detection in local point clouds?
#         )
#     ]
