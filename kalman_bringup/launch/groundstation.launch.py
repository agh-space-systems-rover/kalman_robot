from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ament_index_python import get_package_share_path


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kalman_arm_moveit_config"),
                    "urdf",
                    "arm.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"ignore_timestamp": True}],
        remappings=[
            ("/joint_states", "/arm_controllers/joint_states"),
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_spacenav")
                        / "launch"
                        / "spacenav.launch.py"
                    )
                ),
            ),
            Node(
                package="kalman_master",
                executable="master_com",
                parameters=[{"baud_rate": 38400}],
                # parameters=[{"baud_rate": 115200}],
            ),
            Node(
                package="kalman_arm_utils",
                executable="joint_republisher",
                name="joint_republisher",
            ),
            Node(
                package="kalman_arm_utils",
                executable="gripper_republisher",
                parameters=[{"gripper_scale": 25.0}],
            ),
            Node(
                package="kalman_arm_utils",
                executable="arm_state_republisher",
            ),
            Node(
                package="kalman_master",
                executable="spacenav_driver",
            ),
            Node(
                package="kalman_master",
                executable="wheel_driver",
            ),
            Node(
                package="kalman_master",
                executable="feed_driver",
            ),
            Node(
                package="kalman_master",
                executable="drill_driver",
            ),
            Node(
                package="kalman_master",
                executable="ros_link",
                parameters=[
                    {
                        "config_path": str(
                            get_package_share_path("kalman_arm_config")
                            / "config/ros_link.yaml"
                        ),
                        "side": "station",
                        "rover_endpoint": "arm",
                        "debug_info": True,
                    },
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_wheels")
                        / "launch"
                        / "wheels.launch.py"
                    )
                ),
                launch_arguments={
                    "joy": "arduino",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_bringup")
                        / "launch"
                        / "_commons.launch.py"
                    )
                ),
                launch_arguments={
                    "gs": "true",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    str(get_package_share_path("kalman_arm_config") / "rviz/arm1.rviz"),
                    "--ros-args",
                    "--log-level",
                    "warn",
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    str(get_package_share_path("kalman_arm_config") / "rviz/arm2.rviz"),
                    "--ros-args",
                    "--log-level",
                    "warn",
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    str(get_package_share_path("kalman_arm_config") / "rviz/arm3.rviz"),
                    "--ros-args",
                    "--log-level",
                    "warn",
                ],
            ),
        ]
    )
