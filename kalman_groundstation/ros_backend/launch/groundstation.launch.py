from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python import get_package_share_path

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="kalman_groundstation",
                executable="api_node",
                name="ground_station_api",
            ),
            Node(
                package="kalman_groundstation",
                executable="websocket_node",
                name="ground_station_websocket",
            ),
            Node(
                package="kalman_groundstation",
                executable="bridge_node",
                name="ground_station_bridge",
            ),
            Node(
                package="spacenav",
                executable="spacenav_node",
                name="spacenav",
            ),
            Node(
                package="kalman_master",
                executable="master_com",
                parameters=[{"baud_rate": 38400}],
            ),
            Node(
                package="kalman_arm_utils",
                executable="joint_republisher",
                name="joint_republisher",
            ),
            Node(
                package="kalman_arm_utils",
                executable="gripper_republisher",
            ),
            Node(
                package="spacenav_to_master",
                executable="spacenav_to_master",
            ),
            Node(
                package="wheel_controller",
                executable="wheel_controller",
            ),
            Node(
                package="kalman_master",
                executable="wheel_driver",
            ),
            Node(
                package="kalman_master",
                executable="ros_link",
                parameters=[
                    {
                        "config_path": str(
                            get_package_share_path("kalman_arm_config")
                            / "config/ros_link.yaml"
                        ),
                        "side": "station",
                        "rover_endpoint": "arm",
                    },
                ],
            ),
        ]
    )
