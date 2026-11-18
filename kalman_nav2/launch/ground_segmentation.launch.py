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
import copy

from kalman_utils.launch import launch_node_or_load_component, load_standalone_config

NAV_CONTAINER_NAME = "nav2_container"

def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)

    actions = []
    
    # GSEG 3D for ground segmentation with IMU alligned check S
    # This node does cloud transformation + downsampling + gravity aligment 
    actions += [
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                get_package_share_path("ground_segmentation_ros2")
                / "launch"
                / "ground_segmentation.launch.py"
            )
        ),
        launch_arguments={
            "params_file": str(
                get_package_share_path("kalman_nav2")
                / "config"
                / "ground_seg.yaml"
            ),
            "pointcloud_topic": "/livox/points/cropped",
            "imu_topic": "/livox/imu",
            "use_sim_time": "false",
        }.items(),
    ),
    ]
    #  Node(
    #     package="kalman_nav2",
    #     executable="level_frame",
    #     name="level_frame_node",
    # ),
    # ]
    # actions += [
    #     Node(
    #         package="pcl_ros",
    #         executable="filter_passthrough_node",
    #         name="filter_passthrough_node",
    #         parameters=[
    #             {
    #                 "filter_field_name": "x",
    #                 "filter_limit_min": -100000.0,
    #                 "filter_limit_max": 100000.0,
    #                 "output_frame": "base_link_level",
    #             }
    #         ],
    #         remappings=[
    #             ("input", "/cloud_registered_body"),
    #             ("output", "/cloud_registered_body/transformed"),
    #         ],
    #     )
    # ]
    # # Ground segmentation for obstacle detection
    # actions += launch_node_or_load_component(
    #     component_container=component_container,
    #     package="patchworkpp",
    #     executable="patchworkpp_node",
    #     plugin="patchworkpp_ros::GroundSegmentationServer",
    #     parameters=[
    #         load_standalone_config("kalman_nav2", "patchwork.yaml"),
    #     ],
    #     remappings=[
    #         ("pointcloud_topic", "/cloud_registered_body/transformed"),
    #         ("/patchworkpp/nonground", "/cloud/nonground"),
    #     ],
    # )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="",
                description="Name of an existing component container to use. Empty by default to disable composition.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
