# Drivers for the Master's subsystems. They talk to the Master through the
# master_com topics, so kalman_tunnel's tunnel.launch.py must run alongside
# (kalman_hardware's hardware.launch.py master:=... starts both).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    def get_str(name):
        return LaunchConfiguration(name).perform(context)

    mode = get_str("mode")
    if mode not in ["pc", "gs", "arm"]:
        raise ValueError(
            f'\n\nInvalid Master mode: "{mode}". Please set mode:=... Choose one of: pc, gs, arm'
        )

    # Only the drivers kept on the ground station and the rover. The science,
    # drill and camera_2dof drivers (drill_driver, storage_driver,
    # sand_storage_driver, ph_driver, camera_2dof_driver) are no longer
    # started in any mode; they are still installed for manual `ros2 run`.
    nodes = {
        "ueuos_driver": Node(
            package="kalman_drivers",
            executable="ueuos_driver",
        ),
        "feed_driver": Node(
            package="kalman_drivers",
            executable="feed_driver",
        ),
        "wheel_driver": Node(
            package="kalman_drivers",
            executable="wheel_driver",
        ),
        "autonomy_spammer": Node(
            package="kalman_drivers",
            executable="autonomy_switch_spam",
        ),
        "arm_twist_driver": Node(
            package="kalman_drivers",
            executable="arm_twist_driver",
            remappings={
                "arm/twist": "spacenav/twist",
            }.items(),
        ),
        "tunnel": Node(
            package="kalman_drivers",
            executable="tunnel_client",
        ),
        "laser_driver": Node(
            package="kalman_drivers",
            executable="laser_driver",
        ),
        "estop_driver": Node(
            package="kalman_drivers",
            executable="estop_driver",
        ),
        "rfid_driver": Node(
            package="kalman_drivers",
            executable="rfid_driver",
        ),
    }

    mode_configs = {
        "pc": [
            "wheel_driver",
            "ueuos_driver",
            "autonomy_spammer",
            "laser_driver",
        ],
        "gs": [
            "wheel_driver",
            "ueuos_driver",
            "feed_driver",
            "arm_twist_driver",
            "estop_driver",
            "rfid_driver",
            "tunnel",
            "laser_driver",
        ],
        "arm": [],
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
