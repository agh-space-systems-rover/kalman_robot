from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    deadzone_val = LaunchConfiguration("deadzone").perform(context)
    description = [
        Node(
            package="kalman_arc",
            executable="travel_distance_meter",
            name="travel_distance_meter_node",
            parameters=[{"deadzone": float(deadzone_val)}],
            output="screen",
        )
    ]

    return description


def generate_launch_description():
    rscp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("kalman_arc"), "/launch/rscp.launch.py"]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "deadzone",
                default_value="0.1",
                description="Deadzone in meters below which distance is not tracked.",
            ),
            OpaqueFunction(function=launch_setup),
            rscp_launch,
        ]
    )
