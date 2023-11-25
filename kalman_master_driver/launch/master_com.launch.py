from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    declare_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port to connect to')
    return LaunchDescription([
        declare_port_cmd,
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