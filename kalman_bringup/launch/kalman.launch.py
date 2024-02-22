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


def launch_setup(context):
    unity_sim = LaunchConfiguration("unity_sim").perform(context).lower() == "true"
    gazebo = LaunchConfiguration("gazebo").perform(context).lower() == "true"
    drivers = LaunchConfiguration("drivers").perform(context).lower() == "true"
    rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    rgbd_ids = LaunchConfiguration("rgbd_ids").perform(context)

    if int(unity_sim) + int(gazebo) + int(drivers) > 1:
        raise RuntimeError(
            "Only one of the 'unity_sim', 'gazebo', and 'drivers' arguments can be set to true."
        )

    description = []

    if unity_sim:
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("unity_sim")
                        / "launch"
                        / "unity_sim.launch.py"
                    )
                ),
            ),
        ]

    if gazebo:
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_gazebo")
                        / "launch"
                        / "gazebo.launch.py"
                    )
                ),
            ),
        ]

    if drivers:
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_drivers")
                        / "launch"
                        / "drivers.launch.py"
                    )
                ),
                launch_arguments={"rgbd_ids": rgbd_ids}.items(),
            ),
        ]

    if rviz:
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_bringup")
                        / "launch"
                        / "rviz.launch.py"
                    )
                ),
            ),
        ]

    description += [
        # SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(get_package_share_path("kalman_slam") / "launch" / "slam.launch.py")
            ),
            launch_arguments={"rgbd_ids": rgbd_ids}.items(),
        ),
        # Nav2 stack + path follower
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(get_package_share_path("kalman_nav2") / "launch" / "nav2.launch.py")
            ),
            launch_arguments={"rgbd_ids": rgbd_ids}.items(),
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

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "unity_sim",
                default_value="false",
                description="Start up the Unity simulator with virtual sensors and actuators.",
            ),
            DeclareLaunchArgument(
                "gazebo",
                default_value="false",
                description="Start up the Gazebo simulator with virtual sensors and actuators.",
            ),
            DeclareLaunchArgument(
                "drivers",
                default_value="false",
                description="Launch with physical sensors and actuators.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz.",
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                default_value="d455_front d455_back d455_left d455_right",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
