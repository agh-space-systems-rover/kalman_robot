from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def launch_setup(context):
    def get_bool(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"

    def get_list(name):
        raw_str = LaunchConfiguration(name).perform(context)
        raw_str = raw_str.strip().strip("'\"")
        parts = raw_str.replace(',', ' ').replace('[', ' ').replace(']', ' ')
        return [item.strip().strip("'\"") for item in parts.split() if item.strip()]

    layouts_list = get_list("layouts")
    normalized_layouts = " ".join(layouts_list) if layouts_list else "arm"

    main_xacro_path = str(
        get_package_share_path("kalman_description") / "urdf" / "kalman_main.urdf.xacro"
    )

    urdf = xacro.process_file(
        main_xacro_path,
        mappings={"layouts": normalized_layouts},
    ).toxml()

    description = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf}, {"ignore_timestamp": True}],
        ),
    ]

    parameters = [
        {
            "robot_description": urdf,
            "rate": 30,
            "source_list": [
                "arm_controllers/joint_states",
                "arm/joint_states",
            ],
        },
    ]

    if get_bool("joint_state_publisher_gui"):
        description += [
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=parameters,
            ),
        ]
    else:
        description += [
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                parameters=parameters,
            ),
        ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "layouts",
                default_value="arm",
                description="Space- or comma-separated list of layouts, e.g. 'arm autonomy'.",
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
