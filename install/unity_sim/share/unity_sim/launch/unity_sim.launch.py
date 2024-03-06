from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_path


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="unity_sim", executable="simulation"),
            # rosbridge server for C# <-> ROS communication
            Node(package="rosbridge_server", executable="rosbridge_websocket"),
            # custom Unix Socket server for camera image transfer
            Node(package="unity_rs_publisher", executable="unity_rs_publisher"),
        ]
    )
