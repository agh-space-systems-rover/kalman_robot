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
            "imu": "full",
        },
        "clouds": {
            "rgbd_ids": RGBD_IDS,
        },
        "slam": {
            "rgbd_ids": RGBD_IDS,
            "gps_datum": "50.06378884 19.91458721",  # Test ERC 2024 T1, Kraków Park Jordana
            "fiducials": "terc2024",
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
        "supervisor": {},
    })
