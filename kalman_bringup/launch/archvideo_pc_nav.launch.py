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
                "imu": "full",  # NOTE: Change to "no_mag" to drive without magnetometer.
            },
            "clouds": {
                "rgbd_ids": RGBD_IDS,
            },
            "slam": {
                "rgbd_ids": RGBD_IDS,
                "gps_datum": "50.06623534 19.9132241",  # ERC2024 Marsyard S1, Kraków; NOTE: You can update this to spawn at a different location, but you can reposition the rover using GS map header buttons instead.
                "use_mag": "true",  # NOTE: Disable this to be able to correct the yaw from GS. (This will also have to be disabled once proper SLAM is implemented, but for now it just disables GS updates and sticks to IMU readouts.)
            },
            "nav2": {
                "rgbd_ids": RGBD_IDS,
                "static_map": "erc2024",  # NOTE: Comment-out to only detect obstacles dynamically.
            },
            "wheels": {},
            "supervisor": {},
        }
    )
