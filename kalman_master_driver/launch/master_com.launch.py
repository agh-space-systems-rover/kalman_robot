from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kalman_master_driver",
            executable="master_com",
            name="master_com",
            parameters=[
                {
                    "serial_port": "/dev/ttyUSB0",
                }
            ],
        ),
    ])