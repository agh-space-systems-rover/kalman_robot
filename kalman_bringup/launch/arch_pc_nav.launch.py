from kalman_bringup import *

RGBD_IDS = "d455_front d455_back d455_left d455_right"


def generate_launch_description():
    return gen_launch(
        {
            "description": {
                "layout": "autonomy",
            },
            "hardware": {
                "rgbd_ids": RGBD_IDS,
                "imu": "no_mag",
                "master": "pc",
            },
            "clouds": {
                "rgbd_ids": RGBD_IDS,
            },
            "slam": {
                "rgbd_ids": RGBD_IDS,
                "slam": "d455_front",
            },
            "wheels": {
                "joy": "gamepad",
            },
            "arch": {
                "rgbd_ids": RGBD_IDS,                
            },
            "yolo": {
                "rgbd_ids": RGBD_IDS,
                "config": "arch2025",
            },
        },
        composition="start_container",
    )
