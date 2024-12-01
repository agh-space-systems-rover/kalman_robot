from kalman_bringup import *


def generate_launch_description():
    return gen_launch({
        "hardware": {
            "master": "pc",
            "compass_calibration": "60.0",                
        },
        "wheels": {},
    })
