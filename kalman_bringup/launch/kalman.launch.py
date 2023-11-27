from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_setup(context):
    unity_sim = LaunchConfiguration("unity_sim").perform(context)
    drivers = LaunchConfiguration("drivers").perform(context)
    realsense_ids = LaunchConfiguration("realsense_ids").perform(context)

    if unity_sim == "True" and drivers == "True":
        raise RuntimeError(
            "Cannot launch with both physical drivers and Unity simulation."
        )

    return [
        # -----------------------------------
        # Unity simulation / physical drivers
        # -----------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    get_package_share_path("unity_sim")
                    / "launch"
                    / "unity_sim.launch.py"
                )
            ),
            condition=IfCondition(LaunchConfiguration("unity_sim")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    get_package_share_path("kalman_drivers")
                    / "launch"
                    / "drivers.launch.py"
                )
            ),
            launch_arguments={"realsense_ids": realsense_ids}.items(),
            condition=IfCondition(LaunchConfiguration("drivers")),
        ),
        # ----
        # RViz
        # ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    get_package_share_path("kalman_bringup")
                    / "launch"
                    / "rviz.launch.py"
                )
            ),
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
        # -----
        # stack
        # -----
        # robot structure TF publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    get_package_share_path("kalman_description")
                    / "launch"
                    / "robot_state_publisher.launch.py"
                )
            ),
        ),
        # SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(get_package_share_path("kalman_slam") / "launch" / "slam.launch.py")
            ),
            launch_arguments={"realsense_ids": realsense_ids}.items(),
        ),
        # Nav2 stack + path follower
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(get_package_share_path("kalman_nav2") / "launch" / "nav2.launch.py")
            )
        ),
        # wheel controller
        Node(
            package="kalman_wheel_controller",
            executable="wheel_controller",
            parameters=[
                {
                    "robot_radius": 0.5,
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "unity_sim",
                default_value="False",
                description="Start up the Unity simulator with virtual sensors and actuators.",
            ),
            DeclareLaunchArgument(
                "drivers",
                default_value="False",
                description="Launch with physical sensors and actuators.",
            ),
            DeclareLaunchArgument(
                "realsense_ids",
                default_value="d455_front d455_back d455_left d455_right",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="False",
                description="Launch RViz.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
