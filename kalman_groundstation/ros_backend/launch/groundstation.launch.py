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
    ])