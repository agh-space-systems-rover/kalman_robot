# This launch file includes all modules and it allows to selectively launch them by setting appropriate arguments.
# It allows to quickly create launch files with custom configurations and maintain their readability.
#
# Usage:
# All modules have a "{module}" argument, which is used to enable or disable the module.
# This argument is always false by default.
# If you set "{module}" to "true", please also include all other "{module}.*" arguments.
# If "{module}" is not set, "{module}.*" arguments should not be specified.
# When running multiple launch configurations together, please make sure that each "{module}" argument is set to "true" at most in only one of them.
#
# If any "*.composition" argument is set to "true", please ensure that "component_container" was also set to "true" in this launch file or in any other one that is currently running.
# Since any launch file based on this one is meant to be run by the user, it should only declare its own arguments if necessary. In that case, all arguments should also provide sensible default values.

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


def launch_setup(context):
    def get_bool(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"

    def get_str(name):
        return LaunchConfiguration(name).perform(context)

    description = []

    if get_bool("component_container"):
        description += [
            Node(
                package="rclcpp_components",
                executable="component_container_mt",
                name=COMPONENT_CONTAINER_NAME,
                arguments=["--ros-args", "--log-level", "warn"],
            )
        ]

    if get_bool("rviz"):
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
                    "--ros-args",
                    "--log-level",
                    "warn",
                ],
            ),
        ]

    if get_bool("mapviz"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_mapviz")
                        / "launch"
                        / "mapviz.launch.py"
                    )
                ),
            ),
        ]

    if get_bool("description"):
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
                    "joint_state_publisher_gui": get_str(
                        "description.joint_state_publisher_gui"
                    ),
                }.items(),
            )
        ]

    if get_bool("unity_sim"):
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

    if get_bool("gazebo"):
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

    if get_bool("drivers"):
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
                    "component_container": (
                        COMPONENT_CONTAINER_NAME
                        if get_bool("drivers.composition")
                        else ""
                    ),
                    "master": get_str("drivers.master"),
                    "rgbd_ids": get_str("drivers.rgbd_ids"),
                    "imu": get_str("drivers.imu"),
                    "compasscal": get_str("drivers.compasscal"),
                    "gps": get_str("drivers.gps"),
                }.items(),
            ),
        ]

    if get_bool("clouds"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_clouds")
                        / "launch"
                        / "clouds.launch.py"
                    )
                ),
                launch_arguments={
                    "component_container": (
                        COMPONENT_CONTAINER_NAME
                        if get_bool("clouds.composition")
                        else ""
                    ),
                    "rgbd_ids": get_str("clouds.rgbd_ids"),
                }.items(),
            ),
        ]

    if get_bool("slam"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_slam")
                        / "launch"
                        / "slam.launch.py"
                    )
                ),
                launch_arguments={
                    "component_container": (
                        COMPONENT_CONTAINER_NAME if get_bool("slam.composition") else ""
                    ),
                    "rgbd_ids": get_str("slam.rgbd_ids"),
                    "gps": get_str("slam.gps"),
                    "gps_datum": get_str("slam.gps_datum"),
                    "no_gps_map_odom_offset": get_str("slam.no_gps_map_odom_offset"),
                    "mapping": get_str("slam.mapping"),
                }.items(),
            ),
        ]

    if get_bool("nav2"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_nav2")
                        / "launch"
                        / "nav2.launch.py"
                    )
                ),
                launch_arguments={
                    "component_container": (
                        COMPONENT_CONTAINER_NAME if get_bool("nav2.composition") else ""
                    ),
                    "rgbd_ids": get_str("nav2.rgbd_ids"),
                    "static_map": get_str("nav2.static_map"),
                }.items(),
            ),
        ]

    if get_bool("wheel_controller"):
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

    if get_bool("aruco"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_aruco")
                        / "launch"
                        / "aruco.launch.py"
                    )
                ),
                launch_arguments={
                    "component_container": (
                        COMPONENT_CONTAINER_NAME
                        if get_bool("aruco.composition")
                        else ""
                    ),
                    "rgbd_ids": get_str("aruco.rgbd_ids"),
                }.items(),
            ),
        ]

    if get_bool("yolo"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_yolo")
                        / "launch"
                        / "yolo.launch.py"
                    )
                ),
                launch_arguments={
                    "rgbd_ids": get_str("yolo.rgbd_ids"),
                    "config": get_str("yolo.config"),
                }.items(),
            ),
        ]

    if get_bool("supervisor"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_supervisor")
                        / "launch"
                        / "supervisor.launch.py"
                    )
                ),
                launch_arguments={
                    "aruco_rgbd_ids": get_str("aruco.rgbd_ids") if get_bool("aruco") else "",
                    "yolo_enabled": get_str("yolo"),
                    # NOTE: It is required that kalman_aruco is started from within the same launch file.
                }.items(),
            ),
        ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="false",
                description="Start up the main component container. Must be spawned in order to enable composition.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz.",
            ),
            DeclareLaunchArgument(
                "rviz.config",
                default_value="",
                description="RViz configuration file.",
            ),
            DeclareLaunchArgument(
                "mapviz",
                default_value="false",
                description="Launch MapViz.",
            ),
            DeclareLaunchArgument(
                "description",
                default_value="false",
                description="Start up state publishers for the robot description.",
            ),
            DeclareLaunchArgument(
                "description.joint_state_publisher_gui",
                default_value="false",
                description="Start up the joint state publisher with a GUI.",
            ),
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
                "drivers.composition",
                default_value="false",
                description="Use node composition.",
            ),
            DeclareLaunchArgument(
                "drivers.rgbd_ids",
                default_value="",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            DeclareLaunchArgument(
                "drivers.master",
                default_value="false",
                description="Start the master driver.",
            ),
            DeclareLaunchArgument(
                "drivers.imu",
                default_value="false",
                description="Start the IMU driver.",
            ),
            DeclareLaunchArgument(
                "drivers.compasscal",
                default_value="false",
                description="Start the IMU compass calibration node. IMU must be disabled in order to calibrate the compass.",
            ),
            DeclareLaunchArgument(
                "drivers.gps",
                default_value="false",
                description="Start the GPS driver.",
            ),
            DeclareLaunchArgument(
                "clouds",
                default_value="false",
                description="Generate point clouds from depth cameras.",
            ),
            DeclareLaunchArgument(
                "clouds.composition",
                default_value="false",
                description="Use node composition.",
            ),
            DeclareLaunchArgument(
                "clouds.rgbd_ids",
                default_value="",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="false",
                description="Start up the SLAM module.",
            ),
            DeclareLaunchArgument(
                "slam.composition",
                default_value="false",
                description="Use node composition.",
            ),
            DeclareLaunchArgument(
                "slam.rgbd_ids",
                default_value="",
                description="Space-separated IDs of the depth cameras to use for localization.",
            ),
            DeclareLaunchArgument(
                "slam.gps",
                default_value="false",
                description="Use GPS data to generate map->odom. If disabled, a static transform is used. GPS additionally provides map->utm that allows to send goals in UTM coordinates.",
            ),
            DeclareLaunchArgument(
                "slam.gps_datum",
                default_value="",
                description="The 'latitude longitude' of the map frame. Only used if GPS is enabled. Empty to assume first recorded GPS fix.",
            ),
            DeclareLaunchArgument(
                "slam.no_gps_map_odom_offset",
                default_value="",
                description="The 'x y' translation from map to odom frame. Only used if GPS is disabled. Empty means zero offset.",
            ),
            DeclareLaunchArgument(
                "slam.mapping",
                default_value="false",
                description="Create a 3D point cloud of the terrain as the robot moves.",
            ),
            DeclareLaunchArgument(
                "nav2",
                default_value="false",
                description="Start up the Nav2 stack.",
            ),
            DeclareLaunchArgument(
                "nav2.composition",
                default_value="false",
                description="Use node composition.",
            ),
            DeclareLaunchArgument(
                "nav2.rgbd_ids",
                default_value="",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            DeclareLaunchArgument(
                "nav2.static_map",
                default_value="",
                description="Name of the static map to use. Maps are stored in kalman_nav2/maps. Empty to disable static map.",
            ),
            DeclareLaunchArgument(
                "wheel_controller",
                default_value="false",
                description="Start up the wheel controller.",
            ),
            DeclareLaunchArgument(
                "aruco",
                default_value="false",
                description="Start up ArUco marker tracking.",
            ),
            DeclareLaunchArgument(
                "aruco.composition",
                default_value="false",
                description="Use node composition.",
            ),
            DeclareLaunchArgument(
                "aruco.rgbd_ids",
                default_value="",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            DeclareLaunchArgument(
                "yolo",
                default_value="false",
                description="Start up YOLO object detection.",
            ),
            DeclareLaunchArgument(
                "yolo.rgbd_ids",
                default_value="",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            DeclareLaunchArgument(
                "yolo.config",
                default_value="",
                description="Name of the YOLO configuration file. Configuration files are located in kalman_yolo/config.",
            ),
            DeclareLaunchArgument(
                "supervisor",
                default_value="false",
                description="Start up the supervisor.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
