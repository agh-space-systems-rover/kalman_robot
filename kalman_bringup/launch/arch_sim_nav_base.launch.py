from kalman_bringup import *


def generate_launch_description():
    return gen_launch(
        {
            "unity_sim": {
                "scene": "ARCh2024",
                "selective_launch": "no_rs_pub",
            },
            "rviz": {
                "configs": "rtabmap",
            },
            "gs": {},
            "wheels": {
                "joy": "gamepad",
            },
            "description": {
                "layout": "autonomy",
            },
        },
    )
