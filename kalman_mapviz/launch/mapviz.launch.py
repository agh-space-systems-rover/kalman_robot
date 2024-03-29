from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os
import shutil


def generate_launch_description():
    # Find maproxy-util if it is not in the PATH.
    mapproxy_util = "mapproxy-util"
    if not shutil.which(mapproxy_util):
        mapproxy_util = os.path.expanduser("~/.local/bin/mapproxy-util")
        if not os.path.exists(mapproxy_util):
            raise FileNotFoundError(
                "mapproxy-util not found. Please ensure it is available in PATH or at ~/.local/bin."
            )

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    mapproxy_util,
                    "serve-develop",
                    str(
                        get_package_share_path("kalman_mapviz")
                        / "config"
                        / "mapproxy.yaml"
                    ),
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="mapviz",
                        executable="mapviz",
                        parameters=[
                            {
                                "config": str(
                                    get_package_share_path("kalman_mapviz")
                                    / "config"
                                    / "mapviz.mvc"
                                )
                            }
                        ],
                        arguments=["--ros-args", "--log-level", "error"],
                    ),
                ],
            ),
        ]
    )
