from kalman_bringup import *


def generate_launch_description():
    return gen_launch(
        {
            "unity_sim": {
                "selective_launch": "only_rs_pub",
            },
            # "clouds": {
            #     "rgbd_ids": "d455_arm",
            # },
            # "slam": {
            #     "rgbd_ids": "d455_arm",
            # },
            "arm2": {},
            "aruco":{
                "rgbd_ids": "d435_arm",
                "dict": "5X5_50",
                "size": "0.06",
            },
        },
    )
