# Kalman

ROS 2 Kalman robot software stack

![](./docs/cover.png)

## Prerequisites

- [rtabmap_sync](https://github.com/introlab/rtabmap_ros) built with `-DRTABMAP_SYNC_MULTI_RGBD=ON`

## Getting Started

```bash
ros2 launch kalman_bringup kalman.launch.py
# ros2 launch kalman_bringup kalman.launch.py unity_sim:=true # to also start the simulation
# ros2 launch kalman_bringup kalman.launch.py drivers:=true # to also start the physical drivers
```

Please also note that physical drivers cannot be run along the simulation as they both provide a homogeneous interface to the (simulated or physical) hardware using the same ROS 2 topics.

## Packages

Kalman's software stack is composed of multiple packages that are meant to be built and run together:
- `kalman_bringup` - a set of launch files for the rover and the ground station
- `kalman_description` - a set of Xacro / URDF descriptions + models for the rover
- `kalman_drivers` - a set of drivers for the physical hardware; only to be run separately from the simulation
- `kalman_interfaces` - a set of ROS 2 interfaces used by the other `kalman_` packages
- `kalman_nav2` - a set of configuration files for Nav2
- `kalman_robot` - a metapackage that depends on all other `kalman_` packages
- `kalman_slam` - a set of configuration files for robot_localization and RTAB-Map
- `kalman_wheel_controller` - a controller node that converts Twist messages on `/cmd_vel` and similar topics to the actual wheel state

## Sub-projects

- `service_based_nav2_controller` - a `FollowPath` controller plugin for Nav2 that uses a service to compute velocity commands
- `unity_sim` - a Unity-based simulation environment that can seamlessly replace the physical hardware of AGH Space Systems' robots

## Known Issues; TODOs

- sometimes IMU readings are not taken into account by the UKF until Unity process is restarted
- Nav2 randomly stops planning paths and refuses to acknowledge new goals until restart
- Nav2 often fails to communicate with the action server and unnecessarily begins a recovery phase.
- Nav2 sometimes starts spamming "[ActionServer] Aborting handle." indefinitely after goal is aborted.
