from kalman_bringup import *
from launch_ros.actions import Node

RGBD_IDS = "d455_front d455_back d455_left d455_right"



def generate_launch_description():
    desc = gen_launch(
        {
            "unity_sim": {
                "selective_launch": "only_rs_pub",
            },
            "description": {
                "layout": "autonomy_livox",
            },
        #     "clouds": {
        #         "rgbd_ids": RGBD_IDS,
        #     },
        #     "slam": {
        #         "rgbd_ids": RGBD_IDS,
        #         "gps_datum": "50.06614847 19.91317746",  # ERC 2026 Marsyard S1, Kraków
        #         "fiducials": "erc2026",
        #         "use_mag": "true",
        #     },
        #     "nav2": {
        #         "rgbd_ids": RGBD_IDS,
        #         "static_map": "erc2026",
        #     },
        #     "aruco": {
        #         "rgbd_ids": RGBD_IDS,
        #         "dict": "5X5_100",
        #         "size": "0.15",
        #     },
        #     "supervisor": {},
        # },
        # composition="start_container",
        }
    )
    desc.add_action(
        Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            parameters=[{"ROS_IP": "0.0.0.0"}],
        )
    )
    return desc
