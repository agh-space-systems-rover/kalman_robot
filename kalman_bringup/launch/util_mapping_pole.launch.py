from kalman_bringup import *

RGBD_IDS = "d455_front d455_left"


def generate_launch_description():
    return gen_launch(
        {
            "description": {
                "layout": "autonomy_90deg_cams",
            },
            "hardware": {
                "rgbd_ids": RGBD_IDS,
            },
            "clouds": {
                "rgbd_ids": RGBD_IDS,
            },
            "slam": {
                "rgbd_ids": RGBD_IDS,
                # "slam_rgbd_ids": RGBD_IDS,
                "use_mag": "false",
            },
            "nav2": {
                "rgbd_ids": RGBD_IDS,
            },
            "rviz": {
                "configs": "autonomy",
            },
        },
        composition="start_container",
    )
