# Arm Config

configuration and launch files for the arm

## Description

```
kalman_arm_config
├── config
│   ├── arm_controller.yaml             # Configuration file for the arm controllers (velocity and joint trajectory)
│   ├── cyclonedds.xml                  # Configuration file for the CycloneDDS on the arm
│   ├── nmea_navsat_driver.yaml         # Configuration file for the NMEA GPS driver
|   ├── phidgets_spatial.yaml           # Configuration file for the Phidgets Spatial driver (IMU)
|   ├── predefined_poses.yaml           # Configuration file for predefined arm poses (thresholds and poses list)
|   ├── predefined_trajectories.yaml    # Configuration file for predefined arm trajectories (thresholds and trajectories list)
|   ├── servo_config.yaml               # Configuration file for the MoveIt servo controller
├── launch
│   ├── arm_controller.launch.py        # Launch file for the arm controllers, ROS2 control, state publisher and broadcaster
│   ├── drivers.launch.py               # Launch file for the arm drivers (Phidgets Spatial, NMEA GPS)
│   ├── master.launch.py                # Launch file for the master communications, ROS link and utils
│   ├── servo.launch.py                 # Launch file for the MoveIt servo controller and servo parameter setter
|   ├── trajectories.launch.py          # Launch file for the trajectory and pose executors
```
