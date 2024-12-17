from kalman_bringup import *

RGBD_IDS = "d455_front d455_back d455_left d455_right"

def generate_launch_description():
    return gen_launch({
        "description": {
            "layout": "pc",
        },
        "hardware": {
            "master": "pc",
            "rgbd_ids": RGBD_IDS,
            "imu": "true",
        },
        "clouds": {
            "rgbd_ids": RGBD_IDS,
        },
        "slam": {
            "rgbd_ids": RGBD_IDS,
            "gps_datum": "50.06623534 19.9132241",  # ERC2024 Marsyard S1, Kraków
            "fiducials": "erc2024",
        },
        "nav2": {
            "rgbd_ids": RGBD_IDS,
            "static_map": "erc2024",
        },
        "wheels": {},
        "aruco": {
            "rgbd_ids": RGBD_IDS,
            "dict": "5X5_100",
            "size": "0.15",
        },
        "supervisor": {},
    })