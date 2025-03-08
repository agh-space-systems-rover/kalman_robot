from kalman_bringup import gen_launch

WIFI = True


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
