from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "arm", package_name="kalman_arm_moveit_config"
    ).to_moveit_configs()

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                ],
                output="log",
            )
        ]
    )
