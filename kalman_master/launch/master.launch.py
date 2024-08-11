from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_path


BAUD_RATES = {
    "pc": 115200,
    "gs": 38400,
    "arm": 2000000,
}

PORTS = {"pc": "/tmp/ttyV2"}


def launch_setup(context):
    def get_bool(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"

    def get_str(name):
        return LaunchConfiguration(name).perform(context)

    description = [
        Node(
            package="kalman_master",
            executable="master_com",
            parameters=[
                {
                    "baud_rate": BAUD_RATES[get_str("mode")],
                    "port": PORTS.get(get_str("mode"), ""),  # empty = auto
                }
            ],
        ),
        Node(
            package="kalman_master",
            executable="ros_link",
            parameters=[
                {
                    "config_path": str(
                        get_package_share_path("kalman_master") / "config/ros_link.yaml"
                    ),
                    "side": (
                        "station" if get_str("mode") == "gs" else "rover"
                    ),  # station or rover
                    "rover_endpoint": (
                        "arm" if get_str("mode") == "arm" else "pc"
                    ),  # arm or pc
                },
            ],
        ),
        Node(
            package="kalman_master",
            executable="wheel_driver",
        ),  # Used by autonomy on PC or for teleop on GS.
    ]

    if get_str("mode") == "pc":
        description += [
            Node(
                package="kalman_master",
                executable="autonomy_switch_spam",
            ),
            Node(
                package="kalman_master",
                executable="ueuos_driver",
            ),
        ]

    if get_str("mode") == "gs":
        description += [
            Node(
                package="kalman_master",
                executable="tunnel_client",
            ),
            Node(
                package="kalman_master",
                executable="feed_driver",
            ),
        ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="pc",
                description="Set to 'gs' if master is being run on the ground station or to 'arm' if master is being run on the arm.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
