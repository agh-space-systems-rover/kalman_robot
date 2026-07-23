from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


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
        Node(
            package="kalman_arc",
            executable="tunnel_follower",
        ),
    ]

    if get_bool("enable_rscp_hw_driver") and 0:
        description += [
            Node(
                package="kalman_arc",
                executable="rscp_node",
            ),
        ]
        arc_serial = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(get_package_share_directory("kalman_arc"), "launch"),
                    "/arc_serial.launch.py",
                ]
            )
        )

        description += [
            arc_serial,
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
