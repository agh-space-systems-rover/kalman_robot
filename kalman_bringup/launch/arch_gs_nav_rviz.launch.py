from kalman_bringup import *


def generate_launch_description():
    return gen_launch(
        {
            "rviz": {
                "configs": "rtabmap_wifi",
            },
        },
    )
