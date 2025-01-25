from kalman_bringup import gen_launch

WIFI = False  # NOTE: Set this to use Wi-Fi and see RViz, but I think that RF-only is more stable.


def generate_launch_description():
    return gen_launch(
        {
            "gs": {},
            **(
                {
                    "rviz": {
                        "configs": "autonomy",
                    },
                }
                if WIFI
                else {
                    "hardware": {
                        "master": "gs",
                    },
                    "wheels": {},
                }
            ),
        }
    )
