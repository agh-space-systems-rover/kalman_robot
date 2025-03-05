from kalman_bringup import *

RGBD_IDS = "d455_front d455_back d455_left d455_right"


def generate_launch_description():
    return gen_launch(
        {
            "unity_sim": {
                "selective_launch": "only_ros",
            },
            "clouds": {
                "rgbd_ids": RGBD_IDS,
            },
            "slam": {
                "rgbd_ids": "d455_front",
                "gps_datum": "-34.52825 138.6859",  # EXTERRES Analogue Facility, Roseworthy SA 5371
                "use_mag": "false",
                "slam": "true",
            },
            # "nav2": {
            #     "rgbd_ids": RGBD_IDS,
            # },
            # "supervisor": {},
        },
        composition="start_container",
    )
