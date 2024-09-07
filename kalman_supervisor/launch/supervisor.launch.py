from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_path
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration


def remap_action(from_name, to_name):
    return [
        (f"{from_name}/_action/send_goal", f"{to_name}/_action/send_goal"),
        (f"{from_name}/_action/cancel_goal", f"{to_name}/_action/cancel_goal"),
        (f"{from_name}/_action/feedback", f"{to_name}/_action/feedback"),
        (f"{from_name}/_action/get_result", f"{to_name}/_action/get_result"),
        (f"{from_name}/_action/status", f"{to_name}/_action/status"),
    ]


def launch_setup(context):
    aruco_rgbd_ids = [
        x
        for x in LaunchConfiguration("aruco_rgbd_ids").perform(context).split(" ")
        if x != ""
    ]
    deactivate_aruco = (
        LaunchConfiguration("deactivate_aruco").perform(context).lower() == "true"
    )
    yolo_enabled = (
        LaunchConfiguration("yolo_enabled").perform(context).lower() == "true"
    )

    remappings = []

    for i, rgbd_id in enumerate(aruco_rgbd_ids):
        remappings += [
            (f"aruco/get_state{i}", f"{rgbd_id}/aruco_tracker/get_state"),
            (f"aruco/change_state{i}", f"{rgbd_id}/aruco_tracker/change_state"),
            (f"aruco/detection{i}", f"{rgbd_id}/aruco_detections"),
        ]

    remappings += [
        ("map/map", "global_costmap/costmap"),
        ("map/map_updates", "global_costmap/costmap_updates"),
        ("map/global/set_parameters", "global_costmap/global_costmap/set_parameters"),
        ("map/local/set_parameters", "local_costmap/local_costmap/set_parameters"),
        ("missions/rviz_tf_goal", "supervisor/rviz_tf_goal"),
        *remap_action("missions/tf_goal", "supervisor/tf_goal"),
        *remap_action("missions/gps_goal", "supervisor/gps_goal"),
        *remap_action("missions/gps_aruco_search", "supervisor/gps_aruco_search"),
        *remap_action("missions/gps_yolo_search", "supervisor/gps_yolo_search"),
        *remap_action("nav/navigate_to_pose", "navigate_to_pose"),
        ("ueuos/set_state", "ueuos/set_state"),
        ("yolo/get_state", "yolo_detect/get_state"),
        ("yolo/change_state", "yolo_detect/change_state"),
        ("yolo/detections", "yolo_detections"),
        ("search/path_follower/set_parameters", "path_follower/set_parameters"),
        ("search/path_follower/get_parameters", "path_follower/get_parameters"),
    ]

    return [
        Node(
            package="kalman_supervisor",
            executable="supervisor",
            parameters=[
                str(
                    get_package_share_path("kalman_supervisor")
                    / "config"
                    / "supervisor.yaml"
                ),
                {
                    "aruco": {
                        "enabled": len(aruco_rgbd_ids) > 0,
                        "deactivate_unused": deactivate_aruco,
                        "num_cameras": len(aruco_rgbd_ids),
                    },
                    "yolo": {"enabled": yolo_enabled},
                },
            ],
            remappings=remappings,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "aruco_rgbd_ids",
                description="Space-separated IDs of the depth cameras that were configured in kalman_aruco.",
                default_value="d455_front d455_back d455_left d455_right",
            ),
            DeclareLaunchArgument(
                "deactivate_aruco",
                description="Deactivate ArUco detection nodes when supervisor is not activaly looking for tags.",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "yolo_enabled",
                description="Whether YOLO detection is enabled.",
                default_value="true",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
