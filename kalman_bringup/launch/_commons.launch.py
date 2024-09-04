# This launch file includes all modules and it allows to selectively launch them by setting appropriate arguments.
# It allows to quickly create launch files with custom configurations and maintain their readability.
#
# Usage:
# All modules have a "{module}" argument, which is used to enable or disable the module.
# This argument is always false by default.
# If you set "{module}" to "true", please also consider all other "{module}.*" arguments.
# The default values are always empty strings, false booleans, zero numbers, etc.
# Some of them can be left out, but others might be required.
# If "...{module}" is not set, "...{module}.*" arguments should not be specified.
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
                launch_arguments={
                    "scene": get_str("unity_sim.scene"),
                }.items(),
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
                    "master.mode": get_str("drivers.master.mode"),
                    "rgbd_ids": get_str("drivers.rgbd_ids"),
                    "imu": get_str("drivers.imu"),
                    "compass_calibration": get_str("drivers.compass_calibration"),
                    "compass_calibration.delay": get_str("drivers.compass_calibration.delay"),
                    "compass_calibration.duration": get_str("drivers.compass_calibration.duration"),
                    "compass_calibration.angular_velocity": get_str("drivers.compass_calibration.angular_velocity"),
                    "declination_calibration": get_str("drivers.declination_calibration"),
                    "declination_calibration.delay": get_str("drivers.declination_calibration.delay"),
                    "declination_calibration.duration": get_str("drivers.declination_calibration.duration"),
                    "declination_calibration.velocity": get_str("drivers.declination_calibration.velocity"),
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
                    "gps_datum": get_str("slam.gps_datum"),
                    "fiducials": get_str("slam.fiducials"),
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

    if get_bool("wheels"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_wheels")
                        / "launch"
                        / "wheels.launch.py"
                    )
                ),
                launch_arguments={
                    "joy": get_str("wheels.joy"),
                }.items(),
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
                    "dict": get_str("aruco.dict"),
                    "size": get_str("aruco.size"),
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
                    "aruco_rgbd_ids": (
                        get_str("aruco.rgbd_ids") if get_bool("aruco") else ""
                    ),
                    "deactivate_aruco": get_str("supervisor.deactivate_aruco"),
                    "yolo_enabled": get_str("yolo"),
                    # NOTE: It is required that kalman_aruco is started from within the same launch file.
                }.items(),
            ),
        ]

    if get_bool("gs"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(get_package_share_path("kalman_gs") / "launch" / "gs.launch.py")
                ),
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
                "unity_sim.scene",
                default_value="",
                description="The scene to load in Unity.",
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
                "drivers.master.mode",
                default_value="pc",
                description="Run master drivers in 'pc', 'gs' or 'arm' mode.",
            ),
            DeclareLaunchArgument(
                "drivers.imu",
                default_value="false",
                description="Start the IMU driver.",
            ),
            DeclareLaunchArgument(
                "drivers.compass_calibration",
                default_value="false",
                description="Start the IMU compass calibration service node. IMU must be disabled in order to calibrate the compass.",
            ),
            DeclareLaunchArgument(
                "drivers.compass_calibration.delay",
                default_value="0",
                description="The delay before the node will start the calibration.",
            ),
            DeclareLaunchArgument(
                "drivers.compass_calibration.duration",
                default_value="0",
                description="For how long the robot will keep rotating during compass calibration.",
            ),
            DeclareLaunchArgument(
                "drivers.compass_calibration.angular_velocity",
                default_value="0",
                description="The angular velocity to rotate at during the compass calibration (rad/s).",
            ),
            DeclareLaunchArgument(
                "drivers.declination_calibration",
                default_value="false",
                description="Start the IMU declination calibration node. IMU must be enabled in order to calibrate the declination.",
            ),
            DeclareLaunchArgument(
                "drivers.declination_calibration.delay",
                default_value="0",
                description="The delay before the node will start the calibration.",
            ),
            DeclareLaunchArgument(
                "drivers.declination_calibration.duration",
                default_value="0",
                description="How long the robot will drive forward during declination calibration.",
            ),
            DeclareLaunchArgument(
                "drivers.declination_calibration.velocity",
                default_value="0",
                description="The velocity to drive at during the declination calibration. (m/s)",
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
                "slam.gps_datum",
                default_value="",
                description="The 'latitude longitude' of the map frame. Empty to assume first recorded GPS fix. If set, it will be the initial location of the rover before any readings arrive.",
            ),
            DeclareLaunchArgument(
                "slam.fiducials",
                default_value="",
                description="Collection of fiducials to use for odometry. Empty disables fiducials.",
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
                "wheels",
                default_value="false",
                description="Start up the wheel controller.",
            ),
            DeclareLaunchArgument(
                "wheels.joy",
                default_value="",
                description="Joy device to use for headless driving. Choose 'gamepad' or 'arduino'. Empty disables headless teleop.",
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
                "aruco.dict",
                default_value="",
                description="Dictionary of markers to use.",
            ),
            DeclareLaunchArgument(
                "aruco.size",
                default_value="0",
                description="Size of the markers in meters.",
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
            DeclareLaunchArgument(
                "supervisor.deactivate_aruco",
                default_value="false",
                description="Deactivate ArUco detection nodes when supervisor is not actively looking for tags.",
            ),
            DeclareLaunchArgument(
                "gs",
                default_value="false",
                description="Start up GS.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
