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
    joy = LaunchConfiguration("joy").perform(context).lower()

    if joy != '' and joy != 'gamepad' and joy != 'arduino':
        raise RuntimeError("Invalid joy. Choose 'gamepad' or 'arduino'.")

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

    if joy == 'gamepad':
        description += [
            Node(
                package="joy_linux",
                executable="joy_linux_node",
                parameters=[
                    {
                        "dev_name": "Logitech Gamepad",
                    }
                ],
            ),
            Node(
                package="kalman_wheels",
                executable="gamepad_driving",
                parameters=[get_yaml_params("gamepad_driving")],
            )
        ]
    elif joy == 'arduino':
        description += [
            Node(
                package="joy_linux",
                executable="joy_linux_node",
                parameters=[
                    {
                        "dev_name": "Arduino LLC Arduino Leonardo",
                    }
                ],
            ),
            Node(
                package="kalman_wheels",
                executable="arduino_driving",
                parameters=[get_yaml_params("arduino_driving")],
            )
        ]

    return description

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy",
                default_value="",
                description="Joy device to use for headless driving. Choose 'gamepad' or 'arduino'. Empty disables headless teleop.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
