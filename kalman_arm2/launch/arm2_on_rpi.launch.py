from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="arm_rosbridge_websocket",
                parameters=[
                    {
                        "port": 9473,
                    }
                ],
            ),
        ]
    )
