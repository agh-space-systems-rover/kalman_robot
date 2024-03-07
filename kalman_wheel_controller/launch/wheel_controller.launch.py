from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_path

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kalman_wheel_controller",
            executable="wheel_controller",
            parameters=[
                str(get_package_share_path("kalman_wheel_controller") / "config" / "wheel_controller.yaml"),
            ],
        ),
    ])
