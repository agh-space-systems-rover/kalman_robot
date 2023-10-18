from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import TimerAction


def launch_setup(context):
    return [
        Node(
            namespace=f"{LaunchConfiguration('camera_id').perform(context)}/rgbd_odometry",
            package="rtabmap_odom",
            executable="rgbd_odometry",
            parameters=[
                {
                    "frame_id": "base_link",
                    "publish_tf": False,
                    "subscribe_rgbd": True,
                    "Odom/ResetCountdown": "1",
                }
            ],
            remappings={
                "rgbd_image": f"/{LaunchConfiguration('camera_id').perform(context)}/rgbd_sync/rgbd_image"
            }.items(),
            arguments=["--ros-args", "--log-level", "warn"],
        ),
        # rgbd_sync randomly refuses to work if the camera topics aren't published at the time of startup?
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    namespace=f"{LaunchConfiguration('camera_id').perform(context)}/rgbd_sync",
                    package="rtabmap_sync",
                    executable="rgbd_sync",
                    parameters=[{"approx_sync": False, "queue_size": 50}],
                    remappings={
                        "rgb/image": f"/{LaunchConfiguration('camera_id').perform(context)}/color/image_raw",
                        "depth/image": f"/{LaunchConfiguration('camera_id').perform(context)}/aligned_depth_to_color/image_raw",
                        "rgb/camera_info": f"/{LaunchConfiguration('camera_id').perform(context)}/color/camera_info",
                    }.items(),
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_id",
                default_value="camera",
                description="Prefix of the RGB-D sensor.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
