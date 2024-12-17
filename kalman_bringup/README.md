# Bring-Up

launch files for the whole system

## Launch File Naming

Each launch file is prefixed either with a category or a competition name:
- `main_gs.launch.py`: main ground station setup
- `urc_pc_nav.launch.py`: URC onboard PC setup for the navigation mission
- `util_compasscal.launch.py`: compass calibration utility

In case of competition launches, that prefix (competition name) is followed by the deployment type:
- `urc_gs_...`: ground station laptop
- `urc_pc_...`: onboard PC
- `urc_arm_...`: arm RPi
- `urc_sim_...`: simulated rover and station on the developer's PC

Then there's the name of the mission that the launch file is for:
- `urc_gs_nav`: navigation mission
- `urc_gs_sci`: science mission
...

And then, other details may follow:
- `urc_sim_nav_base`: the base setup for the simulation of URC navigation mission, without the navigation stack
- `urc_sim_nav_stack`: navigation stack, restarted separately from the base setup to accelerate development

## `gen_launch()`

Each launch file uses the `kalman_bringup.gen_launch()` helper to start up different subsystems.

This function takes in a blueprint dict that tells it which modules to start and how to configure them.
It allows us to quickly create launch files with custom configurations while maintaining their readability.

**Example Compass Calibration launch:**

```python
from kalman_bringup import *

def generate_launch_description():
    return gen_launch({
        "hardware": {
            "master": "pc",
            "compass_calibration": "30.0",
        },
        "wheels": {},
    })
```

...which is equivalent to the following commands run in separate shells:

```bash
ros2 launch kalman_hardware hardware.launch.py master:=pc compass_calibration:=30.0
ros2 launch kalman_wheels wheels.launch.py
```
