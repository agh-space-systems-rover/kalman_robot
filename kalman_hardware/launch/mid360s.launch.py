from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare



def launch_setup(context, *args, **kwargs):

    # Retrieve the healthcheck argument
    config_file = LaunchConfiguration("config_file").perform(context)
    # Conditional file creation based on the healthcheck argument
    xfer_format = 0  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    multi_topic = 0  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src = 0  # 0-lidar, others-Invalid data src
    publish_freq = 10.0  # frequency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type = 0
    lvx_file_path = "/home/livox/livox_test.lvx"
    cmdline_bd_code = "livox0000000001"
    user_config_path = PathJoinSubstitution(
        [FindPackageShare("kalman_hardware"), "config", f"{config_file}_config.json"]
    )

    description = []
    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": "livox_frame"},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code},
    ]

    description += [
        Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=livox_ros2_params,
    )
    ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="mid360s",
                description="mid360s config file",
            ),

            OpaqueFunction(function=launch_setup),
        ]
    )