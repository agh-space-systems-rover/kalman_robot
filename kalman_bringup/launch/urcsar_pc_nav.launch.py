from kalman_bringup import *

RGBD_IDS = "d455_front d455_back d455_left d455_right"


def generate_launch_description():
    return gen_launch(
        {
            "description": {
                "layout": "autonomy",
            },
            "hardware": {
                "master": "pc",
                "rgbd_ids": RGBD_IDS,
                "imu": "full",
            },
            "clouds": {
                "rgbd_ids": RGBD_IDS,
            },
            "slam": {
                "rgbd_ids": RGBD_IDS,
                "use_mag": "true",
            },
            "nav2": {
                "rgbd_ids": RGBD_IDS,
            },
            "wheels": {},
            "aruco": {
                "rgbd_ids": RGBD_IDS,
                "dict": "5X5_100",
                "size": "0.15",
            },
            "supervisor": {
                "aruco_rgbd_ids": RGBD_IDS,
                "aruco_deactivate_unused": "true",
            },
        }
    )
