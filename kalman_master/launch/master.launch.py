from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_path


def launch_setup(context):
    def get_bool(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"

    def get_str(name):
        return LaunchConfiguration(name).perform(context)

    return [
        Node(
            package="kalman_master",
            executable="autonomy_switch_spam",
        ),
        Node(
            package="kalman_master",
            executable="master_com",
            parameters=[{"rf_baud": get_str("gs_mode")}],
        ),
        Node(
            package="kalman_master",
            executable="ros_link",
            parameters=[
                {
                    "config_path": str(
                        get_package_share_path("kalman_master") / "config/ros_link.yaml"
                    ),
                    "side": ("gs" if get_bool("gs_mode") else "pc"),
                },
            ],
        ),
        Node(
            package="kalman_master",
            executable="ueuos_driver",
        ),
        Node(
            package="kalman_master",
            executable="wheel_driver",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gs_mode",
                default_value="false",
                description="Set to true if master is being run on the ground station.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
