from kalman_bringup import gen_launch


def generate_launch_description():
    return gen_launch(
        {
            "hardware": {
                "master": "gs",
            },
            "gs": {},
            "wheels": {},
            "rviz": {
                "configs": "autonomy",
            },
        }
    )
