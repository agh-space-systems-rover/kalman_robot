from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    arm_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("kalman_arm"), "launch"),
                "/arm_controller.launch.py",
            ]
        ),
        launch_arguments={"use_sim": "true"}.items()
    )

    master = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("kalman_arm"), "launch"),
                "/master.launch.py",
            ]
        ),
        launch_arguments={"use_sim": "true"}.items()
    )

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("kalman_arm"), "launch"),
                "/servo.launch.py",
            ]
        )
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("kalman_arm_moveit_config"), "launch"
                ),
                "/move_group.launch.py",
            ]
        )
    )

    trajectories = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("kalman_arm"), "launch"),
                "/trajectories.launch.py",
            ]
        )
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("kalman_description"), "launch"),
                "/description.launch.py",
            ]
        ),
        launch_arguments={"layout": "arm"}.items()
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("kalman_rviz"), "launch"),
                "/rviz.launch.py",
            ]
        ),
        launch_arguments={"configs": "arm3"}.items()
    )
    return LaunchDescription(
        [arm_controller, master, servo, move_group, trajectories, description, rviz]
    )
