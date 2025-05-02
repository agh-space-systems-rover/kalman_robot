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
                "slam_rgbd_ids": RGBD_IDS,
                "use_mag": "false",
            },
            "nav2": {
                "rgbd_ids": RGBD_IDS,
                # "driving_mode": "forward",
            },
            "supervisor": {
                "arch_camera_ids": RGBD_IDS,
            },
            "arch": {
                "rgbd_ids": RGBD_IDS,
                "hd_cameras": "true",
            },
            "yolo": {
                "rgbd_ids": RGBD_IDS,
                "config": "arch2025",
            },
            "wheels": {
                # "joy": "gamepad",
            },
        },
        composition="start_container",
    )
