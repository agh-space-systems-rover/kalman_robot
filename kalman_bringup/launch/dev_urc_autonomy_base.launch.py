from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(get_package_share_path("kalman_bringup") / "launch" / "_commons.launch.py")
            ),
            launch_arguments={
                "composition": "true",
                "rgbd_ids": "d455_front d455_back d455_left d455_right",
                "component_container.spawn": "true",
                "unity_sim.spawn": "true",
                "rviz.spawn": "true",
                "rviz.config": "urc_autonomy.rviz"
            }.items(),
        ),
    ])
