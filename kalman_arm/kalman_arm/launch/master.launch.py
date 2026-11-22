from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_path


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")

    argument_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Whether to run in simulation mode without hardware interfaces",
    )

    container = ComposableNodeContainer(
        name="master_node_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="kalman_arm_controller",
                plugin="arm_master::MasterToServo",
                name="master_to_servo_node",
            ),
            ComposableNode(
                package="kalman_arm_controller",
                plugin="kalman_arm::ExtraCanNode",
                name="extra_can_node",
                parameters=[
                    {"max_gripper": 2600, "min_gripper": 1370, "start_pose": 2450}
                ],
            ),
        ],
        output="screen",
        # arguments = ['--ros-args', '--log-level', 'DEBUG', '--log-level','rcl:=INFO'],
    )

    # Serial transport (master_com) and radio link (ros_link) to the GS.
    tunnel = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path("kalman_tunnel") / "launch/tunnel.launch.py")
        ),
        launch_arguments={"mode": "arm", "port": "/dev/ttyAMA1"}.items(),
        condition=UnlessCondition(use_sim),
    )

    arm_state_publisher_node = Node(
        package="kalman_arm_utils",
        executable="arm_state_publisher",
        parameters=[{"rate": 5.0}],
    )

    return LaunchDescription(
        [
            argument_use_sim,
            tunnel,
            arm_state_publisher_node,
            container,
        ]
    )