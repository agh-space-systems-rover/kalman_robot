from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_path


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="kalman_master",
                executable="master_loopback",
                parameters=[
                    {"loss_rate": 0.5},
                ],
            ),
            Node(
                package="kalman_master",
                executable="ros_link",
                name="ros_link_pc",
                parameters=[
                    {
                        "config_path": str(
                            get_package_share_path("kalman_master")
                            / "config/ros_link.yaml"
                        ),
                        "side": "rover",  # station or rover
                        "rover_endpoint": "pc",  # arm or pc
                        "loopback_mangling": True,  # Adds /loopback to all names exposed on the other side.
                        "debug_info": False,  # Info-logs status while running.
                    },
                ],
                output="screen",  # for debug print() statements
            ),
            Node(
                package="kalman_master",
                executable="ros_link",
                name="ros_link_gs",
                parameters=[
                    {
                        "config_path": str(
                            get_package_share_path("kalman_master")
                            / "config/ros_link.yaml"
                        ),
                        "side": "station",
                        "loopback_mangling": True,
                        "debug_info": False,
                    },
                ],
                output="screen",
            ),
        ]
    )
