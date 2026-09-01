from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_path


def generate_launch_description():

    gps_node = Node(
        package="nmea_navsat_driver",
        executable="nmea_serial_driver",
        parameters=[
            str(
                get_package_share_path("kalman_arm")
                / "config"
                / "nmea_navsat_driver.yaml"
            )
        ],
        remappings=[
            ("fix", "gps/fix"),
            ("heading", "gps/heading"),
            ("vel", "gps/vel"),
            ("time_reference", "gps/time_reference"),
        ],
        respawn=True,
        respawn_delay=30,
    )

    return LaunchDescription(
        [
            gps_node,
        ]
    )
