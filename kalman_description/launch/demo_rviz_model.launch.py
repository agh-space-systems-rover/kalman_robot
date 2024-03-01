from ament_index_python import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_description")
                        / "launch"
                        / "robot_state_publisher.launch.py"
                    )
                )
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    str(
                        get_package_share_path("kalman_description")
                        / "rviz"
                        / "demo_rviz_model.rviz"
                    ),
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
        ]
    )
