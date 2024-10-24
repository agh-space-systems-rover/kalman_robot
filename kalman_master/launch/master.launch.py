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
            executable="wheel_driver",
        ),  # Used by autonomy on PC or for teleop on GS.
    ]

    if get_str("mode") == "pc":
        description += [
            Node(
                package="kalman_master",
                executable="autonomy_switch_spam",
            ),
        ]

    if get_bool("autonomy"):
        description += [
            Node(
                package="kalman_master",
                executable="ros_link",
                parameters=[
                    {
                        "config_path": str(
                            get_package_share_path("kalman_master")
                            / "config/pc_ros_link.yaml"
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
        ]

    if get_bool("drivers.arm"):
        description += [
            Node(
                package="kalman_master",
                executable="ros_link",
                parameters=[
                    {
                        "config_path": str(
                            get_package_share_path("kalman_master")
                            / "config/arm_ros_link.yaml"
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
                executable="spacenav_driver",
            ),
        ]

    if get_bool("drivers.ueuos"):
        description += [
            Node(
                package="kalman_master",
                executable="ueuos_driver",
            ),
        ]

    if get_bool("drivers.feed"):
        description += [
            Node(
                package="kalman_master",
                executable="feed_driver",
            ),
        ]

    if get_bool("drivers.tunnel"):
        description += [
            Node(
                package="kalman_master",
                executable="tunnel_client",
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
            DeclareLaunchArgument(
                "autonomy",
                default_value="false",
                description="Set to 'true' to enable autonomy. Starts ros_link and wheel_driver.",
            ),
            DeclareLaunchArgument(
                "drivers.arm",
                default_value="false",
                description="Set to 'true' to enable arm-specific drivers. Starts ros_link and wheel_driver.",
            ),
            DeclareLaunchArgument(
                "drivers.ueuos",
                default_value="false",
                description="Set to 'true' to enable ueuos driver.",
            ),
            DeclareLaunchArgument(
                "drivers.feed",
                default_value="false",
                description="Set to 'true' to enable feed driver.",
            ),
            DeclareLaunchArgument(
                "drivers.tunnel",
                default_value="false",
                description="Set to 'true' to enable tunnel client.",
            ),
            DeclareLaunchArgument(
                "drivers.drill",
                default_value="false",
                description="Set to 'true' to enable drill driver.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
