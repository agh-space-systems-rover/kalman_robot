from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("kalman_arm")

    config_file_path = os.path.join(pkg_dir, "config", "can_link.yaml")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=config_file_path,
        description="Path to the YAML configuration file",
    )

    can_interface_arg = DeclareLaunchArgument(
        "can_interface", default_value="vcan0", description="CAN interface to use"
    )

    container = ComposableNodeContainer(
        name="can_bridge_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="kalman_arm_controller",
                plugin="configurable_can_bridge::CANBridge",
                name="can_bridge",
                parameters=[
                    {
                        "config_file": LaunchConfiguration("config_file"),
                        "can_interface": LaunchConfiguration("can_interface"),
                    }
                ],
            ),
        ],
        output="screen",
        # arguments = ['--ros-args', '--log-level', 'DEBUG', '--log-level','rcl:=INFO'],
    )

    return LaunchDescription([config_file_arg, can_interface_arg, container])
