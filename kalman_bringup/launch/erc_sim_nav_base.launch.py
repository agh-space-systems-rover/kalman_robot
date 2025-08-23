from kalman_bringup import *


def generate_launch_description():
    return gen_launch({
        "unity_sim": {
            "scene": "ERC2024",
            "selective_launch": "no_rs_pub",
        },
        "rviz": {
            "configs": "autonomy",
        },
        "gs": {},
        "wheels": {},
    })
