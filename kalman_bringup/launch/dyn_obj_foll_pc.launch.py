from kalman_bringup import *

RGBD_IDS = "d455_front"


def generate_launch_description():
    return gen_launch(
        {
            "description": {
                "layout": "autonomy",
            },
            "hardware": {
                "master": "pc",
                "rgbd_ids": RGBD_IDS,
                "imu": "no_mag",
            },
            "clouds": {
                "rgbd_ids": RGBD_IDS,
            },
            "slam": {
                "rgbd_ids": RGBD_IDS,
            },
            "nav2": {
                "rgbd_ids": RGBD_IDS,
                "driving_mode": "forward",
            },
            "supervisor": {
                "arch_camera_ids": RGBD_IDS,
            },
            "aruco": {
                "rgbd_ids": RGBD_IDS,
                "dict": "5X5_100",
                "size": "0.15",
            },
            "wheels": {
                "joy": "gamepad",
            },
            "experimental": {},
        },
        composition="start_container",
    )
