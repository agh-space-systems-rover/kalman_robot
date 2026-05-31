from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
def launch_setup(context):
    deadzone_val = LaunchConfiguration("deadzone").perform(context)
    description = [
        Node(
            package="kalman_arc",
            executable="travel_distance_meter",
            name="travel_distance_meter_node",
            parameters=[{"deadzone": float(deadzone_val)}],
            output="screen"
        )
    ]

    return description

def generate_launch_description():
    return LaunchDescription(
        [
        DeclareLaunchArgument(
                "deadzone",
                default_value="0.01",
                description="Deadzone in meters below which distance is not tracked.",
            ),
        OpaqueFunction(function=launch_setup),
        ]
    )
