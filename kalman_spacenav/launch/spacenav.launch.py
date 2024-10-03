from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Create /tmp/silent-run.sh, which will:
    # Run in bash
    # Run exec 1>/dev/null 2>/dev/null
    # Run the command passed as argument
    with open("/tmp/kalman/spacenav", "w") as f:
        f.write("#!/bin/bash\n")
        f.write("exec 1>/dev/null 2>/dev/null\n")
        f.write("$@\n")

    # Make /tmp/silent-run.sh executable
    os.chmod("/tmp/kalman/spacenav", 0o755)

    return LaunchDescription(
        [
            Node(
                package="spacenav",
                executable="spacenav_node",
                prefix="/tmp/kalman/spacenav",
            ),
        ]
    )
