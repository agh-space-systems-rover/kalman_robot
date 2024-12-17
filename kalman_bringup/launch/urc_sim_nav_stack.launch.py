from ament_index_python import get_package_share_path
from kalman_bringup import gen_launch

RGBD_IDS = "d455_front d455_back d455_left d455_right"

def generate_launch_description():
    return gen_launch({
        "description": {
            "layout": "autonomy",
        },
        "clouds": {
            "rgbd_ids": RGBD_IDS,
        },
        "slam": {
            "rgbd_ids": RGBD_IDS,
            "gps_datum": "50.8780616423677 20.642475756324",  # Marsyard S1, Kielce
        },
        "nav2": {
            "rgbd_ids": RGBD_IDS,
        },
        "aruco": {
            "rgbd_ids": RGBD_IDS,
            "dict": "4X4_50",
            "size": "0.15",
        },
        "yolo": {
            "rgbd_ids": RGBD_IDS,
            "config": "urc2024",
        },
        "supervisor": {
            "aruco_rgbd_ids": RGBD_IDS,
            "aruco_deactivate_unused": "true",
            "yolo_enabled": "true",
            "yolo_deactivate_unused": "true",
        },
    })
