from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
import jinja2
import yaml
import os


def load_local_ukf_config(rgbd_ids):
    # Load UKF config template
    with open(
        str(
            get_package_share_path("kalman_slam2")
            / "config"
            / "ukf_filter_node_local.yaml.j2"
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
    ukf_params_path = "/tmp/kalman/ukf_filter_node_local." + \
        str(os.getpid()) + ".yaml"
    os.makedirs(os.path.dirname(ukf_params_path), exist_ok=True)
    with open(ukf_params_path, "w") as f:
        yaml.dump(ukf_params, f)

    # Return path to UKF config
    return ukf_params_path


def launch_setup(context):
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]
    description = []

    description += [
        Node(
            namespace=f"{camera_id}",
            package="rtabmap_odom",
            executable="rgbd_odometry",
            parameters=[
                    str(
                        get_package_share_path("kalman_slam2")
                        / "config"
                        / "rgbd_odometry.yaml"
                    )
            ],
            remappings=[
                ("rgb/image", f"color/image_raw"),
                ("depth/image", f"aligned_depth_to_color/image_raw"),
                ("rgb/camera_info", f"color/camera_info"),
            ],
            arguments=["--ros-args", "--log-level", "error"],
        )
        for camera_id in rgbd_ids
    ]

    description += [
        Node(
            package="robot_localization",
            executable="ukf_node",
            name="ukf_filter_node",
            parameters=[load_local_ukf_config(rgbd_ids)],
            remappings=[("odometry/filtered", "odometry/local")],
        ),
    ]
    return description


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
                default_value="",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
