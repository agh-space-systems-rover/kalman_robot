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
        str(
            get_package_share_path("kalman_gazebo")
            / "urdf"
            / "kalman_with_gazebo.urdf.xacro"
        )
    ).toxml()

    description = [
        # robot structure TF publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
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
        # wheel driver
        Node(package="kalman_gazebo", executable="gazebo_wheel_driver"),
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

    # Run state broadcaster (a dependency of trajectory controller?)
    # and the velocity controller once the robot is spawned.
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

    # Run the trajectory controller after joint state broadcaster is ready.
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
