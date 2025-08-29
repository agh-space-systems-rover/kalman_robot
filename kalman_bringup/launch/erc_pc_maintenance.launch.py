from kalman_bringup import *


def generate_launch_description():
    return gen_launch(
        {
            "description": {
                "layout": "arm_autonomy",
            },
            "hardware": {
                "master": "pc",
                "rgbd_ids": "d435_arm",
                "imu": "full",
            },
            "aruco":{
                "rgbd_ids": "d435_arm",
                "dict": "5X5_50",
                "size": "0.06",
            },
            "wheels": {},
            "arm2": {},
        }
    )
