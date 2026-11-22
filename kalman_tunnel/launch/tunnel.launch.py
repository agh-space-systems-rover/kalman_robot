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

# Which ros_link sides to start in each mode: (side, rover_endpoint).
ROS_LINKS = {
    "pc": [("rover", "pc")],
    "gs": [("station", "pc"), ("station", "arm")],
    "arm": [("rover", "arm")],
}


def start_ros_link(side: str, rover_endpoint: str) -> Node:
    return Node(
        name=f"ros_link_"
        + (
            "gs_to_" + rover_endpoint
            if side == "station"
            else rover_endpoint + "_to_gs"
        ),
        package="kalman_tunnel",
        executable="ros_link",
        parameters=[
            {
                "config_path": str(
                    get_package_share_path("kalman_tunnel")
                    / f"config/{rover_endpoint}_ros_link.yaml"
                ),
                "side": side,  # station or rover
                "rover_endpoint": rover_endpoint,  # arm or pc
            },
        ],
    )


def launch_setup(context):
    def get_str(name):
        return LaunchConfiguration(name).perform(context)

    mode = get_str("mode")
    if mode not in BAUD_RATES:
        raise ValueError(
            f'\n\nInvalid tunnel mode: "{mode}". Please set mode:=... Choose one of: pc, gs, arm'
        )

    master_com = Node(
        package="kalman_tunnel",
        executable="master_com",
        parameters=[
            {
                "baud_rate": BAUD_RATES[mode],
                "port": get_str("port"),  # empty = auto
            }
        ],
    )

    return [master_com] + [
        start_ros_link(side, rover_endpoint)
        for side, rover_endpoint in ROS_LINKS[mode]
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="gs",
                choices=["gs", "pc", "arm"],
                description="On what hardware is this module being run? Available modes: gs, pc, arm",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="",
                description="Serial port of the Master. Leave empty to find it automatically by its USB model.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
