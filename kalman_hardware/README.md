# Hardware

drivers, tools and launch scripts for the physical hardware onboard

> [!WARNING]
> Only to be run separately from the simulation on a physical robot.

This module provides access to the physical hardware of the robot by exposing the same API as the simulation.
All other modules in the software stack use this API to perform their actions independently of whether they are running on the physical robot or in simulation.

## Launch Arguments

- `component_container` (default: ""): Name of an existing component container to use. Empty by default to disable composition.
- `master` (default: ""): Start the master driver in a given mode ('pc', 'gs' or 'arm'). Leave empty to disable.
- `rgbd_ids` (default: ""): Space-separated IDs of the depth cameras to use (d455_front, d455_back, ...). Leave empty to disable the cameras.
- `imu` (default: false): Start the IMU driver.
- `compass_calibration` (default: 0.0): Start IMU compass calibration node for a given number of seconds. IMU must be disabled in order to calibrate the compass. Zero to run in normal mode, without calibration.
- `gps` (default: false): Start the GPS driver.
