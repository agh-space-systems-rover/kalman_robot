from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_path
import os
import shutil

# for DEBUG wheel_controller
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
                        get_package_share_path("kalman_gs") / "config" / "mapproxy.yaml"
                    ),
                    "-blocalhost:8065",
                ],
            ),
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="gs_rosbridge_websocket",
                parameters=[
                    {
                        "port": 9065,
                        "send_action_goals_in_new_thread": True,
                    }
                ],
                ros_arguments=["--ros-args", "--log-level", "fatal"],
                # Bleeding edge rosbridge_server emits a lot of errors when actions are used. It works nevertheless.
            ),
            Node(
                package="kalman_gs",
                executable="gs",
            ),
        ]
    )
