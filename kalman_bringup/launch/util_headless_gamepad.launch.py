from kalman_bringup import *

RGBD_IDS = "d455_front d455_back d455_left d455_right"


def generate_launch_description():
    return gen_launch(
        {
            "hardware": {
                "master": "pc",
            },
            "wheels": {
                "joy": "gamepad"
            },
        }
    )
