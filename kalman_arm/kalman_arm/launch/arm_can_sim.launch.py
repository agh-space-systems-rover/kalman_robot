import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    argument_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Whether to launch RViz with the MoveIt MotionPlanning plugin",
    )

    # This executable uses SocketCAN directly and does not accept ROS arguments.
    simulator = ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix("kalman_arm_controller"),
                "lib", "kalman_arm_controller", "arm_can_simulator",
            ),
            "can0",
        ],
        output="screen",
    )

    arm_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("kalman_arm"), "launch"),
                "/arm_controller.launch.py",
            ]
        ),
        # Exercise the real CAN driver, not the bus-free ArmSimSystem plugin.
        launch_arguments={"use_sim": "false"}.items(),
    )

    position_mode = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once", "--wait-matching-subscriptions", "1",
            "/change_control_type", "std_msgs/msg/UInt8", "{data: 0}",
        ],
        output="screen",
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("kalman_arm_moveit_config"), "launch"
                ),
                "/move_group.launch.py",
            ]
        )
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("kalman_arm_moveit_config"), "launch"
                ),
                "/moveit_rviz.launch.py",
            ]
        ),
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    def start_moveit(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Failed to select arm position mode"))]
        return [move_group, rviz]

    return LaunchDescription(
        [
            argument_rviz,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=simulator,
                    on_start=[arm_controller, position_mode],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=position_mode, on_exit=start_moveit)
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=simulator,
                    on_exit=[EmitEvent(event=Shutdown(reason="CAN simulator exited"))],
                )
            ),
            simulator,
        ]
    )
