# Arm Config

configuration and launch files for the arm

## MoveIt and RViz with the CAN simulator

Prepare an active virtual CAN interface named `can0` separately (see the
`kalman_arm_controller` README). The launch file does not create or configure
network interfaces. Do not use a physical arm CAN bus for this simulation.

Inside the `kalman_ws` distrobox, source ROS and the workspace, then run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kalman_arm arm_can_sim.launch.py
```

This starts the CAN simulator, the real CAN-backed ros2_control hardware plugin,
robot-state publisher, joint-state broadcaster, trajectory and velocity
controllers, MoveIt move_group, and RViz with the MotionPlanning plugin. It waits
for a control-mode subscriber and publishes position mode before starting MoveIt.
Master/UART communications and Servo are intentionally not launched.

Use `rviz:=false` for headless operation. The CAN interface is `can0`, matching
the current hardware driver. Stop with Ctrl+C; the virtual interface is left
untouched.

## Description

```
kalman_arm_config
├── config
│   ├── arm_controller.yaml             # Configuration file for the arm controllers (velocity and joint trajectory)
│   ├── cyclonedds.xml                  # Configuration file for the CycloneDDS on the arm
|   ├── predefined_poses.yaml           # Configuration file for predefined arm poses (thresholds and poses list)
|   ├── predefined_trajectories.yaml    # Configuration file for predefined arm trajectories (thresholds and trajectories list)
|   ├── servo_config.yaml               # Configuration file for the MoveIt servo controller
├── launch
│   ├── arm_controller.launch.py        # Launch file for the arm controllers, ROS2 control, state publisher and broadcaster
│   ├── master.launch.py                # Launch file for the master communications, ROS link and utils
│   ├── servo.launch.py                 # Launch file for the MoveIt servo controller and servo parameter setter
|   ├── trajectories.launch.py          # Launch file for the trajectory and pose executors
```
