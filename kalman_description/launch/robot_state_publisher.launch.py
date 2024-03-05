from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    return LaunchDescription(
        [
            # robot structure TF publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": xacro.process_file(
                            str(
                                get_package_share_path("kalman_description")
                                / "urdf"
                                / "kalman.urdf.xacro"
                            )
                        ).toxml()
                    }
                ],
            ),
            # joint state publisher
            # Required for 3D model joints to show up.
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
            ),
        ]
    )
