from kalman_bringup import *


def generate_launch_description():
    return gen_launch(
        {
            "unity_sim": {
                "scene": "Testing",
                "selective_launch": "no_rs_pub",
            },
            "description": {
                "layout": "arm_autonomy",
            },
            "rviz": {
                "configs": "arm_dev",
            },
        },
    )
