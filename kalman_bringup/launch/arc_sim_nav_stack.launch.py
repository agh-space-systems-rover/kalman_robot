from ament_index_python import get_package_share_path
from kalman_bringup import gen_launch

RGBD_IDS = "d455_front d455_back d455_left d455_right"


def generate_launch_description():
    return gen_launch(
        {
            "unity_sim": {
                "selective_launch": "only_rs_pub",
            },
            "description": {
                "layout": "autonomy",
            },
            "clouds": {
                "rgbd_ids": RGBD_IDS,
            },
            "slam": {
                "rgbd_ids": RGBD_IDS,
                "gps_datum": "41.10512613 29.02332318",  # 424F+384 Sarıyer, İstanbul, Türkiye
            },
            "nav2": {
                "rgbd_ids": RGBD_IDS,
            },
            "aruco": {
                "rgbd_ids": RGBD_IDS,
                "dict": "ARUCO_ORIGINAL",
                "size": "0.20",
            },
            # "yolo": {
            #     "rgbd_ids": RGBD_IDS,
            #     "config": "urc2024",
            # },
            "supervisor": {
                "aruco_rgbd_ids": RGBD_IDS,
                # "aruco_deactivate_unused": "true",
                # "yolo_enabled": "true",
                # "yolo_deactivate_unused": "true",
                "rscp_enabled": "true",
            },
            "arc": {
                "enable_rscp_hw_driver": "false",
            },
        }
    )
