from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_path

def remap_action(from_name, to_name):
    return [
        (f"{from_name}/_action/send_goal", f"{to_name}/_action/send_goal"),
        (f"{from_name}/_action/cancel_goal", f"{to_name}/_action/cancel_goal"),
        (f"{from_name}/_action/feedback", f"{to_name}/_action/feedback"),
        (f"{from_name}/_action/get_result", f"{to_name}/_action/get_result"),
        (f"{from_name}/_action/status", f"{to_name}/_action/status"),
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="kalman_supervisor",
                executable="supervisor",
                parameters=[str(
                    get_package_share_path("kalman_supervisor")
                    / "config" / "supervisor.yaml"
                )],
                remappings=[
                    ("map/map", "global_costmap/costmap"),
                    ("map/map_updates", "global_costmap/costmap_updates"),
                    ("map/global/set_parameters", "global_costmap/global_costmap/set_parameters"),
                    ("map/local/set_parameters", "local_costmap/local_costmap/set_parameters"),
                    ("missions/rviz_tf_goal", "supervisor/rviz_tf_goal"),
                    *remap_action("missions/tf_goal", "supervisor/tf_goal"),
                    *remap_action("nav/navigate_to_pose", "navigate_to_pose"),
                    ("ueuos/set_state", "ueuos/set_state"),
                ]
            )
        ]
    )
