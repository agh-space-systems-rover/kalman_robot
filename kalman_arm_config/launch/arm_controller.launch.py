# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_path


def generate_launch_description():
    # Get URDF via xacro
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

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("kalman_arm_config"),
            "config",
            "arm_controller.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    robot_vel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner, robot_vel_controller_spawner],
            )
        )
    )


    gps_node = Node(
                package="nmea_navsat_driver",
                executable="nmea_serial_driver",
                parameters=[
                    str(
                        get_package_share_path("kalman_drivers")
                        / "config"
                        / "nmea_navsat_driver.yaml"
                    )
                ],
                remappings=[
                    ("fix", "gps/fix"),
                    ("heading", "gps/heading"),
                    ("vel", "gps/vel"),
                    ("time_reference", "gps/time_reference"),
                ],
                respawn=True,
                respawn_delay=30,
            )

    phidgets_spatial_calibration_params_path = os.path.abspath(
            os.path.join(
                os.path.expanduser("~"),
                ".config/kalman/phidgets_spatial_calibration_params.yaml",
            )
        )
        

    imu_container = ComposableNodeContainer(
            name='imu_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            respawn=True,
            composable_node_descriptions=[
                ComposableNode(
                    package="phidgets_spatial",
                    plugin="phidgets::SpatialRosI",
                    parameters=[
                        str(
                            get_package_share_path("kalman_drivers")
                            / "config"
                            / "phidgets_spatial.yaml"
                        ),
                        phidgets_spatial_calibration_params_path,
                    ],
                    # NOTE: Spatial does not support intra-process communication.
                        
                ),
                ComposableNode(
                    package="imu_filter_madgwick",
                    plugin="ImuFilterMadgwickRos",
                    parameters=[
                        str(
                            get_package_share_path("kalman_drivers")
                            / "config"
                            / "imu_filter_madgwick.yaml"
                        ),
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
            output='both',
    )

    

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        gps_node,
        imu_container,
    ]

    return LaunchDescription(nodes)
