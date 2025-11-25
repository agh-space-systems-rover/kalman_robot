from kalman_bringup import gen_launch

def generate_launch_description():
    return gen_launch(
        {
            "gs": {},
            "rviz": {
                "configs": "autonomy",
            },
        }
    )
