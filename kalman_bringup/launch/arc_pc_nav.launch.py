from kalman_bringup import *

RGBD_IDS = "d455_front d455_back d455_left d455_right"


def generate_launch_description():
    return gen_launch(
        {
            "description": {
                "layout": "autonomy_90deg_cams",
            },
            "hardware": {
                "master": "pc",
                "rgbd_ids": RGBD_IDS,
                "imu": "full",
                "gps": "true",
            },
            "clouds": {
                "rgbd_ids": RGBD_IDS,
            },
            "slam": {
                "rgbd_ids": RGBD_IDS,
                "gps_datum": "41.10512613 29.02332318",  # 424F+384 Sarıyer, İstanbul, Türkiye CHANGE TO POLAND ON TEST!
            },
            "nav2": {
                "rgbd_ids": RGBD_IDS,
            },
            "aruco": {
                "rgbd_ids": RGBD_IDS,
                "dict": "ARUCO_ORIGINAL",
                "size": "0.20",
            },
            #  "yolo": {
            #     "rgbd_ids": "d455_right",
            #     "config": "arc2026",
            # },
            "wheels": {},
            "supervisor": {
                "rscp_enabled": "true",
                "aruco_rgbd_ids": RGBD_IDS,
            },
            "arc": {
                "enable_rscp_hw_driver": "true",
            },
        }
    )
