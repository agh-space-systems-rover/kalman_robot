from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_prefix
import os


def generate_launch_description():
    throttle_output = (
        get_package_prefix("kalman_spacenav") + "/lib/kalman_spacenav/throttle_output"
    )

    os.makedirs("/tmp/kalman", exist_ok=True)
    with open("/tmp/kalman/spacenav", "w") as f:
        f.write("#!/bin/bash\n")
        f.write(throttle_output + " 10 $@\n")

    # Make /tmp/silent-run.sh executable
    os.chmod("/tmp/kalman/spacenav", 0o755)

    return LaunchDescription(
        [
            Node(
                package="spacenav",
                executable="spacenav_node",
                output="screen",
                prefix="/tmp/kalman/spacenav",
            ),
        ]
    )
