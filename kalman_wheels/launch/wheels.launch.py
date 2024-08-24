from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_path

def get_yaml_params(name: str) -> str:
    return str(
        get_package_share_path("kalman_wheels") / "config" / f"{name}.yaml"
    )

def launch_setup(context):
    headless_joy = LaunchConfiguration("headless_joy").perform(context).lower() == "true"
    arduino_joy = LaunchConfiguration("arduino_joy").perform(context).lower() == "true"

    description = [
        Node(
            package="kalman_wheels",
            executable="twist_controller",
            parameters=[get_yaml_params("twist_controller")],
        ),
        Node(
            package="kalman_wheels",
            executable="drive_controller",
            parameters=[get_yaml_params("drive_controller")],
        ),
    ]

    if headless_joy:
        description += [
            Node(
                package="joy_linux",
                executable="joy_linux_node",
                parameters=[
                    {
                        "default_trig_val": True,
                    }
                ]
            ),
            Node(
                package="kalman_wheels",
                executable="joy_driving",
                parameters=[get_yaml_params("joy_driving")],
            )]
    if arduino_joy:    
        description += [    
            Node(
                package="joy",
                executable="joy_node",
                parameters=[
                    {
                        "device_name": "Arduino LLC Arduino Leonardo",
                    }
                ],
                remappings=[
                    ("/joy", "/arduino/joy"),
                ]
            ),
            Node(
                package="kalman_wheels",
                executable="arduino_driving",
                remappings=[
                    ("/joy", "/arduino/joy"),
                ]
            )
        ]

    return description

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless_joy",
                default_value="false",
                description="Launch headless gamepad controller.",
            ),
            DeclareLaunchArgument(
                "arduino_joy",
                default_value="true",
                description="Launch arduino gamepad controller.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
