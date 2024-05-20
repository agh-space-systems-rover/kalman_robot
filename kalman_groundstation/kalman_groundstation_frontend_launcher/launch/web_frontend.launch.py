from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # This is not a node, but ROS does not care
        Node(
            package='kalman_groundstation_frontend_launcher',
            executable='launcher',
            name='kalman_groundstation_frontend_launcher',
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')]
        ),
    ])
