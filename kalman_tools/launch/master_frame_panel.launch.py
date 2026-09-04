from importlib.util import find_spec

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    start_rosbridge = LaunchConfiguration("start_rosbridge").perform(context)
    rosbridge_port = LaunchConfiguration("rosbridge_port").perform(context)
    streamlit_port = LaunchConfiguration("streamlit_port").perform(context)
    rosbridge_host = LaunchConfiguration("rosbridge_host").perform(context)

    app_spec = find_spec("kalman_tools.apps.master_frame_panel")
    if app_spec is None or app_spec.origin is None:
        raise RuntimeError("Could not locate kalman_tools.apps.master_frame_panel")
    app_path = app_spec.origin

    actions = []

    if start_rosbridge.lower() in ("true", "1", "yes"):
        actions.append(
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="kalman_tools_rosbridge_websocket",
                parameters=[
                    {
                        "port": int(rosbridge_port),
                        "send_action_goals_in_new_thread": True,
                    }
                ],
                ros_arguments=["--ros-args", "--log-level", "warn"],
            )
        )

    actions.append(
        ExecuteProcess(
            cmd=[
                "streamlit",
                "run",
                app_path,
                "--server.headless",
                "true",
                "--server.port",
                streamlit_port,
            ],
            additional_env={
                "KALMAN_TOOLS_ROSBRIDGE_HOST": rosbridge_host,
                "KALMAN_TOOLS_ROSBRIDGE_PORT": rosbridge_port,
            },
            output="screen",
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_rosbridge",
                default_value="true",
                description="Start a local rosbridge_websocket instance.",
            ),
            DeclareLaunchArgument(
                "rosbridge_port",
                default_value="3001",
                description="WebSocket port for rosbridge (GS uses 9065).",
            ),
            DeclareLaunchArgument(
                "streamlit_port",
                default_value="8501",
                description="HTTP port for the Streamlit panel.",
            ),
            DeclareLaunchArgument(
                "rosbridge_host",
                default_value="localhost",
                description="Default rosbridge host passed to the Streamlit app.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
