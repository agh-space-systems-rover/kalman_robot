from kalman_bringup import *

RGBD_IDS = "d455_front d455_back d455_left d455_right"


def generate_launch_description():
    return gen_launch(
        {
            "unity_sim": {
                "selective_launch": "only_ros",
            },
            "clouds": {
                "rgbd_ids": RGBD_IDS,
            },
            "slam": {
                "rgbd_ids": RGBD_IDS,
                "slam": "d455_front",
            },
            "nav2": {
                "rgbd_ids": RGBD_IDS,
                "driving_mode": "forward",
            },
            "supervisor": {},
        },
        composition="start_container",
    )
