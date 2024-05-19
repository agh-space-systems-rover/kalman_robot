from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
        )
    ])