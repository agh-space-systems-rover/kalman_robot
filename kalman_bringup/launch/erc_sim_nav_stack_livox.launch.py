from kalman_bringup import *
from launch_ros.actions import Node


def generate_launch_description():
    desc = gen_launch(
        {
            "unity_sim": {
                "selective_launch": "only_rs_pub",
            },
            "description": {
                "layout": "autonomy_livox",
            },
            "slam": {
                "lio_config": "mid360s_sim",
                "gps_datum": "50.06614847 19.91317746",  # ERC 2026 Marsyard S1, Kraków
                "use_mag": "true",
                "use_rtabmap": "false", # enable loop closure detection
            },
            "nav2": {
                # "static_map": "erc2026",
            },
        #     "aruco": {
        #         "rgbd_ids": RGBD_IDS,
        #         "dict": "5X5_100",
        #         "size": "0.15",
        #     },
            "supervisor": {},
        # },
        },
        composition="start_container",
    )
    desc.add_action(
        Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            parameters=[{"ROS_IP": "0.0.0.0"}],
        )
    )
    return desc
