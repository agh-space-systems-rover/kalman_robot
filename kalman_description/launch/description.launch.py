from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
import xacro


def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)
    joint_state_publisher_gui = LaunchConfiguration("joint_state_publisher_gui").perform(context).lower() == "true"

    urdf = xacro.process_file(
        str(
            get_package_share_path("kalman_description")
            / "urdf"
            / "kalman.urdf.xacro"
        )
    ).toxml()

    description = []

    # robot structure TF publisher
    if component_container:
        description += [
            LoadComposableNodes(
                target_container=component_container,
                composable_node_descriptions=[
                    ComposableNode(
                        package="robot_state_publisher",
                        plugin="robot_state_publisher::RobotStatePublisher",
                        parameters=[
                            {
                                "robot_description": urdf
                            }
                        ],
                        # NOTE: Intra-Process communication is not supported.
                    )
                ],
            )
        ]
    else:
        description += [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": urdf
                    }
                ],
            ),
        ]

    # joint state publisher
    # Required for 3D model joints to show up.
    # This node is written in Python and is not composable.
    if not joint_state_publisher_gui:
        description += [
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
            ),
        ]

    # alternative joint state publisher with GUI
    if joint_state_publisher_gui:
        description += [
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
        ]

    return description

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="", description="Name of an existing component container to use. Empty by default to disable composition."
            ),
            DeclareLaunchArgument(
                "joint_state_publisher_gui", default_value="true", description="Start the master driver."
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
