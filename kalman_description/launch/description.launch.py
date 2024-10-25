from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context):
    def get_bool(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"

    if get_bool("with_arm"):
        urdf = xacro.process_file(
            str(
                get_package_share_path("kalman_description") / "urdf" / "arm.urdf.xacro"
            )
        ).toxml()
    else:
        urdf = xacro.process_file(
            str(
                get_package_share_path("kalman_description")
                / "urdf"
                / "kalman.urdf.xacro"
            )
        ).toxml()

    # robot structure TF publisher
    description = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf}, {"ignore_timestamp": True}],
        ),
    ]

    # alternative joint state publisher with GUI
    if get_bool("joint_state_publisher_gui"):
        description += [
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
        ]
    # joint state publisher
    # Required for 3D model joints to show up.
    # This node is written in Python and is not composable.
    else:
        description += [
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                parameters=[
                    {"rate": 10},
                    {
                        "source_list": (
                            ["/arm_controllers/joint_states"]
                            if get_bool("with_arm")
                            else []
                        )
                    },
                ],
            ),
        ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joint_state_publisher_gui",
                default_value="true",
                description="Start the joint state publisher in GUI mode.",
            ),
            DeclareLaunchArgument(
                "with_arm",
                default_value="false",
                description="Set robot description to include the arm.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
