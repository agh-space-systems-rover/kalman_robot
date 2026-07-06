from kalman_bringup import *


def generate_launch_description():
    return gen_launch(
        {
            "unity_sim": {
                "scene": "ARC2024",
                "selective_launch": "no_rs_pub",
            },
            "rviz": {
                "configs": "arc_sim_dev",
            },
            "gs": {},
            "wheels": {},
        }
    )
