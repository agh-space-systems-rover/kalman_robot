from ament_index_python import get_package_share_path

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    robot_description = xacro.process_file(
        str(get_package_share_path("kalman_description") / "urdf" / "kalman.urdf.xacro")
    ).toxml()

    description = [
        # robot state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    get_package_share_path("kalman_description")
                    / "launch"
                    / "robot_state_publisher.launch.py"
                )
            ),
        ),
        Node(package="joint_state_publisher", executable="joint_state_publisher"),
        Node(package="kalman_gazebo", executable="gazebo_wheel_driver"),
        # Gazebo simulator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    get_package_share_path("ros_ign_gazebo")
                    / "launch"
                    / "ign_gazebo.launch.py"
                )
            ),
            launch_arguments=[("gz_args", [" -r -v 4 shapes.sdf"])],
        ),
    ]

    # Spawn the robot in Gazebo.
    gazebo_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description,
            "-name",
            "kalman",
            "-allow_renaming",
            "true",
            "-z",
            "1.5",
            "-x",
            "-2",
        ],
    )
    description += [gazebo_spawn]

    # Run controllers after the robot is spawned.
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
    )
    load_velocity_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "velocity_controller",
        ],
    )
    description += [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo_spawn,
                on_exit=[load_joint_state_broadcaster, load_velocity_controller],
            )
        )
    ]

    # Run trajectory controller after joint state broadcaster.
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_trajectory_controller",
        ],
    )
    description += [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        )
    ]

    # load_imu_sensor_broadcaster = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "control",
    #         "load_controller",
    #         "--set-state",
    #         "active",
    #         "imu_sensor_broadcaster",
    #     ],
    #     output="screen",
    # )

    return LaunchDescription(description)
