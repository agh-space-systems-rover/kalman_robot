import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('kalman_arm_controller2')
    config_file = os.path.join(pkg_share, 'config', 'gripper.yaml')

    can_iface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='SocketCAN interface (can0 or vcan0)'
    )

    gripper_node = Node(
        package='kalman_arm_controller2',
        executable='gripper_node',
        name='gripper_node',
        output='screen',
        parameters=[
            config_file,
            {'can_interface': LaunchConfiguration('can_interface')}
        ],
    )

    return LaunchDescription([
        can_iface_arg,
        gripper_node,
    ])