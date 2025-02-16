from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_path
import os
import shutil
import yaml


def generate_launch_description():
    # Find maproxy-util if it is not in the PATH.
    mapproxy_util = "mapproxy-util"
    if not shutil.which(mapproxy_util):
        mapproxy_util = os.path.expanduser("~/.local/bin/mapproxy-util")
        if not os.path.exists(mapproxy_util):
            raise FileNotFoundError(
                "mapproxy-util not found. Please ensure it is available in PATH or at ~/.local/bin."
            )

    # Load config/mapproxy.yaml and insert cache dir.
    with open(
            str(get_package_share_path("kalman_gs") / "config" / "mapproxy.yaml")
    ) as f:
        mapproxy_yaml = yaml.load(f, Loader=yaml.SafeLoader)
    mapproxy_yaml["globals"] = {}
    mapproxy_yaml["globals"]["cache"] = {}
    mapproxy_yaml["globals"]["cache"]["base_dir"] = os.path.expanduser(
        "~/.cache/kalman/mapproxy_cache"
    )

    # Save mapproxy config to a temp file.
    mapproxy_yaml_path = "/tmp/kalman/mapproxy." + str(os.getpid()) + ".yaml"
    os.makedirs(os.path.dirname(mapproxy_yaml_path), exist_ok=True)
    with open(mapproxy_yaml_path, "w") as f:
        yaml.dump(mapproxy_yaml, f)

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    mapproxy_util,
                    "serve-develop",
                    mapproxy_yaml_path,
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
            Node(
                package="kalman_gs",
                executable="autonomy_config",
                parameters=[
                    {
                        "config_path": str(
                            get_package_share_path("kalman_master")
                            / f"config/autonomy_status.yaml"
                        ),
                    }
                ]
            ),
        ]
    )
