from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
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

    master_uart_node = Node(
        package="kalman_master",
        executable="master_com",
        parameters=[{"baud_rate": 2000000, "port": "/dev/ttyAMA2"}],
        condition=UnlessCondition(use_sim),
    )

    arm_state_publisher_node = Node(
        package="kalman_arm_utils",
        executable="arm_state_publisher",
        parameters=[{"rate": 5.0}],
    )

    ros_link_node = Node(
        package="kalman_master",
        executable="ros_link",
        parameters=[
            {
                "config_path": str(
                    get_package_share_path("kalman_master") / "config/arm_ros_link.yaml"
                ),
                "side": "rover",
                "rover_endpoint": "arm",
            },
        ],
        condition=UnlessCondition(use_sim),
    )

    return LaunchDescription(
        [
            argument_use_sim,
            master_uart_node,
            ros_link_node,
            arm_state_publisher_node,
            container,
        ]
    )