from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_path


BAUD_RATES = {
    "pc": 115200,
    "gs": 38400,
    "arm": 2000000,
}

PORTS = {"pc": "/tmp/ttyV2"}


def start_ros_link(side: str, rover_endpoint: str) -> Node:
    return Node(
        package="kalman_master",
        executable="ros_link",
        parameters=[
            {
                "config_path": str(
                    get_package_share_path("kalman_master")
                    / f"config/{rover_endpoint}_ros_link.yaml"
                ),
                "side": side,  # station or rover
                "rover_endpoint": rover_endpoint,  # arm or pc
            },
        ],
        name=f"ros_link_{rover_endpoint}"
    )


def launch_setup(context):
    def get_str(name):
        return LaunchConfiguration(name).perform(context)

    mode = get_str("mode")
    if mode not in ["pc", "gs", "arm"]:
        raise ValueError(
            f'\n\nInvalid Master mode: "{mode}". Please set mode:=... Choose one of: pc, gs, arm'
        )

    nodes = {
        "master_com": Node(
            package="kalman_master",
            executable="master_com",
            parameters=[
                {
                    "baud_rate": BAUD_RATES[get_str("mode")],
                    "port": PORTS.get(get_str("mode"), ""),  # empty = auto
                }
            ],
        ),
        "ueuos_driver": Node(
            package="kalman_master",
            executable="ueuos_driver",
        ),
        "feed_driver": Node(
            package="kalman_master",
            executable="feed_driver",
        ),
        "wheel_driver": Node(
            package="kalman_master",
            executable="wheel_driver",
        ),
        "autonomy_spammer": Node(
            package="kalman_master",
            executable="autonomy_switch_spam",
        ),
        "autonomy_status": Node(
            package="kalman_master",
            executable="status_publisher",
            parameters=[
                {
                    "config_path": str(
                        get_package_share_path("kalman_master")
                        / f"config/autonomy_status.yaml"
                    ),
                }
            ]
        ),
        "link_pc_to_gs": start_ros_link(side="rover", rover_endpoint="pc"),
        "link_arm_to_gs": start_ros_link(side="rover", rover_endpoint="arm"),
        "link_gs_to_pc": start_ros_link(side="station", rover_endpoint="pc"),
        "link_gs_to_arm": start_ros_link(side="station", rover_endpoint="arm"),
        "arm_twist_driver": Node(
            package="kalman_master",
            executable="arm_twist_driver",
            remappings={
                "arm/twist": "spacenav/twist",
            }.items(),
        ),
        "tunnel_client": Node(
            package="kalman_master",
            executable="tunnel_client",
        ),
        "drill_driver": Node(
            package="kalman_master",
            executable="drill_driver",
        ),
        "rfid_driver": Node(
            package="kalman_master",
            executable="rfid_driver",
        )
    }

    mode_configs = {
        "pc": [
            "master_com",
            "link_pc_to_gs",
            "wheel_driver",
            "ueuos_driver",
            "autonomy_spammer",
            "tunnel_client",
            "autonomy_status",
        ],
        "gs": [
            "master_com",
            "link_gs_to_pc",
            "link_gs_to_arm",
            "wheel_driver",
            "ueuos_driver",
            "feed_driver",
            "arm_twist_driver",
            "drill_driver",
            "rfid_driver",
        ],
        "arm": [
            "master_com",
            "link_arm_to_gs",
        ],
    }

    return [nodes[node_name] for node_name in mode_configs[mode]]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="gs",
                choices=["gs", "pc", "arm"],
                description="On what hardware is this module being run? Available modes: gs, pc, arm",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
