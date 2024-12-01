from kalman_bringup import *

def generate_launch_description():
    return gen_launch({
        "description": {
            "layout": "arm",
        },
        "hardware": {
            "master": "gs",
        },
        "wheels": {
            "joy": "arduino",
        },
        "gs": {},
        "rviz": {
            "configs": "arm1 arm2 arm3",
        },
        "arm_utils": {},
        "spacenav": {},
    })
