from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def get_layouts():
    return [
        x.name.split(".")[0]
        for x in (get_package_share_path("kalman_description") / "layouts").glob(
            "*.urdf.xacro"
        )
    ]


def launch_setup(context):
    def get_bool(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"

    def get_str(name):
        return LaunchConfiguration(name).perform(context)

    urdf = xacro.process_file(
        str(
            get_package_share_path("kalman_description")
            / "layouts"
            / f"{get_str('layout')}.urdf.xacro"
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

    if get_bool("joint_state_publisher_gui"):
        # alternative joint state publisher with GUI
        description += [
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[
                    {"rate": 10},
                    {"source_list": ["arm_controllers/joint_states"]},
                ],
            ),
        ]
    else:
        # joint state publisher
        # Required for 3D model joints to show up.
        description += [
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                parameters=[
                    {"rate": 10},
                    {"source_list": ["arm_controllers/joint_states"]},
                ],
            ),
        ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "layout",
                choices=[*get_layouts()],
                description="layout of the robot: autonomy, arm",
            ),
            DeclareLaunchArgument(
                "joint_state_publisher_gui",
                default_value="false",
                choices=["true", "false"],
                description="Start the joint state publisher in GUI mode.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
