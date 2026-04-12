import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_launch_dir = os.path.join(
        get_package_share_directory("kalman_bringup"), "launch"
    )

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, "arm_dev_sim_base.launch.py")
        )
    )

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, "arm_dev_sim_stack.launch.py")
        )
    )

    return LaunchDescription([base, stack])
