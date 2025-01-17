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
                "imu": "no_mag",
            },
            "clouds": {
                "rgbd_ids": RGBD_IDS,
            },
            "slam": {
                "rgbd_ids": RGBD_IDS,
                "gps_datum": "FIXME",  # FIXME
            },
            "nav2": {
                "rgbd_ids": RGBD_IDS,
            },
            "wheels": {},
            "supervisor": {},
        }
    )
