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

COMPONENT_CONTAINER_NAME = "kalman_container"
COMPOSITION_BY_DEFAULT = "true"
DEFAULT_RVIZ_CONFIG = "default.rviz"
DEFAULT_RGBD_IDS = "d455_front d455_back d455_left d455_right"

def launch_setup(context):
    def get_bool(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"
    
    def get_str(name):
        return LaunchConfiguration(name).perform(context)
    
    component_container = COMPONENT_CONTAINER_NAME if get_bool("composition") else ""

    description = []

    if get_bool("component_container.spawn"):
        description += [
            Node(
                package='rclcpp_components',
                executable='component_container_mt',
                name=COMPONENT_CONTAINER_NAME,
                arguments=["--ros-args", "--log-level", "warn"],
            )
        ]

    if get_bool("description.spawn"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_description")
                        / "launch"
                        / "description.launch.py"
                    )
                ),
                launch_arguments={
                    "component_container": component_container,
                    "joint_state_publisher_gui": get_str("description.joint_state_publisher_gui"),
                }.items(),
            )
        ]

    if get_bool("unity_sim.spawn"):
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

    if get_bool("gazebo.spawn"):
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

    if get_bool("drivers.spawn"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_drivers")
                        / "launch"
                        / "drivers.launch.py"
                    )
                ),
                launch_arguments={
                    "component_container": component_container,
                    "master": get_str("drivers.master"),
                    "rgbd_ids": get_str("rgbd_ids"),
                    "imu": get_str("drivers.imu"),
                    "compasscal": get_str("drivers.compasscal"),
                }.items(),
            ),
        ]

    if get_bool("rviz.spawn"):
        description += [
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    str(
                        get_package_share_path("kalman_bringup")
                        / "rviz"
                        / get_str("rviz.config")
                    ),
                    "--ros-args", "--log-level", "warn"
                ],
            ),
        ]

    if get_bool("slam.spawn"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(get_package_share_path("kalman_slam") / "launch" / "slam.launch.py")
                ),
                launch_arguments={
                    "component_container": component_container,
                    "rgbd_ids": get_str("rgbd_ids"),
                    "mapping": get_str("slam.mapping"),
                }.items(),
            ),
        ]

    if get_bool("nav2.spawn"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(get_package_share_path("kalman_nav2") / "launch" / "nav2.launch.py")
                ),
                launch_arguments={
                    "component_container": component_container,
                    "rgbd_ids": get_str("rgbd_ids")
                }.items(),
            ),
        ]

    if get_bool("wheel_controller.spawn"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_wheel_controller")
                        / "launch"
                        / "wheel_controller.launch.py"
                    )
                ),
            ),
        ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "composition",
                default_value=COMPOSITION_BY_DEFAULT,
                description="Enable node composition.",
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                default_value=DEFAULT_RGBD_IDS,
                description="Space-separated IDs of the depth cameras to use.",
            ),
            DeclareLaunchArgument(
                "component_container.spawn",
                default_value="false",
                description="Start up the main component container. Must be spawned in order to enable composition.",
            ),
            DeclareLaunchArgument(
                "description.spawn",
                default_value="false",
                description="Start up state publishers for the robot description.",
            ),
            DeclareLaunchArgument(
                "description.joint_state_publisher_gui",
                default_value="false",
                description="Start up the joint state publisher with a GUI.",
            ),
            DeclareLaunchArgument(
                "unity_sim.spawn",
                default_value="false",
                description="Start up the Unity simulator with virtual sensors and actuators.",
            ),
            DeclareLaunchArgument(
                "gazebo.spawn",
                default_value="false",
                description="Start up the Gazebo simulator with virtual sensors and actuators.",
            ),
            DeclareLaunchArgument(
                "drivers.spawn",
                default_value="false",
                description="Launch with physical sensors and actuators.",
            ),
            DeclareLaunchArgument(
                "drivers.master",
                default_value="true",
                description="Start the master driver.",
            ),
            DeclareLaunchArgument(
                "drivers.imu",
                default_value="true",
                description="Start the IMU driver.",
            ),
            DeclareLaunchArgument(
                "drivers.compasscal",
                default_value="false",
                description="Start the IMU compass calibration node. IMU must be disabled in order to calibrate the compass.",
            ),
            DeclareLaunchArgument(
                "rviz.spawn",
                default_value="false",
                description="Launch RViz.",
            ),
            DeclareLaunchArgument(
                "rviz.config",
                default_value=DEFAULT_RVIZ_CONFIG,
                description="RViz configuration file.",
            ),
            DeclareLaunchArgument(
                "slam.spawn",
                default_value="false",
                description="Start up the SLAM module.",
            ),
            DeclareLaunchArgument(
                "slam.mapping",
                default_value="false",
                description="Create a 3D point cloud of the terrain as the robot moves.",
            ),
            DeclareLaunchArgument(
                "nav2.spawn",
                default_value="false",
                description="Start up the Nav2 stack.",
            ),
            DeclareLaunchArgument(
                "wheel_controller.spawn",
                default_value="false",
                description="Start up the wheel controller.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
