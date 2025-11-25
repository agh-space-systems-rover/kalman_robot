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
            "aruco": {
                "rgbd_ids": RGBD_IDS,
                "dict": "5X5_100",
                "size": "0.15",
            },
            "slam": {
                "rgbd_ids": RGBD_IDS,
                "gps_datum": "50.06622974 19.91320048",  # ERC2025 Marsyard S1, Kraków
                "fiducials": "erc2025",
                "use_mag": "true",
            },
            "nav2": {
                "rgbd_ids": RGBD_IDS,
                "static_map": "erc2025",
            },
            "wheels": {},
            "supervisor": {},
        }
    )
