from kalman_bringup import *

RGBD_IDS = "d455_front d455_back d455_left d455_right"


def generate_launch_description():
    return gen_launch(
        {
            "description": {
                "layout": "dev_pole",
            },
            "hardware": {
                "rgbd_ids": RGBD_IDS,
            },
            "clouds": {
                "rgbd_ids": RGBD_IDS,
            },
            "slam": {
                "rgbd_ids": RGBD_IDS,
                "slam": "d455_front",
            },
            "rviz": {
                "configs": "rtabmap",
            },
        },
        composition="start_container",
    )
