from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    def get_bool(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"

    deadzone_val = LaunchConfiguration("travel_distance_meter_deadzone").perform(
        context
    )
    description = [
        Node(
            package="kalman_arc",
            executable="travel_distance_meter",
            name="travel_distance_meter_node",
            parameters=[{"deadzone": float(deadzone_val)}],
            output="screen",
        ),
        Node(
            package="kalman_arc",
            executable="darkest_rock_finder",
        ),
        Node(
            package="kalman_arc",
            executable="peak_finder",
            name="peak_finder_node",
            parameters=[{"map_resolution": 0.3}],
            output="screen",
        ),
    ]

    if get_bool("enable_rscp_hw_driver"):
        description += [
            Node(
                package="kalman_arc",
                executable="rscp_node",
            ),
        ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "travel_distance_meter_deadzone",
                default_value="0.1",
                description="Deadzone in meters below which distance is not tracked.",
            ),
            DeclareLaunchArgument(
                "enable_rscp_hw_driver",
                default_value="false",
                description="Run the RSCP protocol driver. Only use it on real hardware.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
