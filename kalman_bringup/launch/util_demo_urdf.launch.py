from kalman_bringup import *

LAYOUT = "arm"

def generate_launch_description():
    return gen_launch({
        "description": {
            "layout": LAYOUT,
            "joint_state_publisher_gui": "true",
        },
        "rviz": {
            "configs": "demo_urdf_model",
        },
    })
