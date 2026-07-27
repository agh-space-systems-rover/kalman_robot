from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path
from lifecycle_msgs.msg import Transition
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

    udp_receiver = LifecycleNode(
        package="udp_driver",
        executable="udp_receiver_node_exe",
        name="gps_udp_receiver",
        namespace="",
        parameters=[
            {
                "ip": "0.0.0.0",
                "port": ParameterValue(
                    LaunchConfiguration("udp_gps_port"), value_type=int
                ),
            }
        ],
        remappings=[("udp_read", "/gps/udp_packets")],
    )

    configure_udp_receiver = RegisterEventHandler(
        OnProcessStart(
            target_action=udp_receiver,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(udp_receiver),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )

    activate_udp_receiver = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=udp_receiver,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(udp_receiver),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "udp_gps_port",
                default_value="62137",
                description="UDP port used by the GPS JSON sender.",
            ),
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
            udp_receiver,
            configure_udp_receiver,
            activate_udp_receiver,
            Node(
                package="kalman_gs",
                executable="udp_gps_republisher",
                name="udp_gps_republisher",
            ),
            Node(
                package="kalman_gs",
                executable="gs",
            ),
            Node(
                package="kalman_gs",
                executable="pkg_config_reader",
            ),
        ]
    )
