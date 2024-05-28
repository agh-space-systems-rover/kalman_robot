from launch import LaunchDescription
from launch_ros.actions import Node

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
            ('/joint_states', '/arm_controllers/joint_states'),
        ]
    )


    return LaunchDescription(
        [
            robot_state_publisher,
            Node(
                package="kalman_groundstation",
                executable="api_node",
                name="ground_station_api",
            ),
            Node(
                package="kalman_groundstation",
                executable="websocket_node",
                name="ground_station_websocket",
            ),
            Node(
                package="kalman_groundstation",
                executable="bridge_node",
                name="ground_station_bridge",
            ),
            Node(
                package="spacenav",
                executable="spacenav_node",
                name="spacenav",
            ),
            Node(
                package="kalman_master",
                executable="master_com",
                parameters=[{"baud_rate": 38400}],
            ),
            Node(
                package="kalman_arm_utils",
                executable="joint_republisher",
                name="joint_republisher",
            ),
            Node(
                package="kalman_arm_utils",
                executable="gripper_republisher",
                parameters=[{"gripper_scale": 15.0}],
            ),
            Node(
                package="kalman_arm_utils",
                executable="arm_state_republisher",
            ),
            Node(
                package="spacenav_to_master",
                executable="spacenav_to_master",
            ),
            Node(
                package="wheel_controller",
                executable="wheel_controller",
            ),
            Node(
                package="kalman_master",
                executable="wheel_driver",
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
            Node(
                package='kalman_groundstation_frontend_launcher',
                executable='launcher',
                name='kalman_groundstation_frontend_launcher',
                output='screen',
                emulate_tty=True,
                arguments=[('__log_level:=debug')]
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    str(
                        get_package_share_path("kalman_arm_config")
                        / "rviz/arm1.rviz"
                    ),
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
                    str(
                        get_package_share_path("kalman_arm_config")
                        / "rviz/arm2.rviz"
                    ),
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
                    str(
                        get_package_share_path("kalman_arm_config")
                        / "rviz/arm3.rviz"
                    ),
                    "--ros-args",
                    "--log-level",
                    "warn",
                ],
            ),
        ]
    )
