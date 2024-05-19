from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, Node


def generate_launch_description():
    
    spacenav = Node(
        package="spacenav_node",
        executable="spacenav_node",
        output="screen",
    )
    
    twist_republisher = Node(
        package="kalman_arm_controller",
        executable="twist_republisher",
        output="screen",
    )
    
    return LaunchDescription(
        [
            spacenav,
            twist_republisher,
        ]
    )
