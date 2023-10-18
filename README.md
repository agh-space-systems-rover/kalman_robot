# Kalman

ROS 2 Kalman robot software stack

![](./docs/cover.png)

## Prerequisites

- [rtabmap_sync](https://github.com/introlab/rtabmap_ros) built with `-DRTABMAP_SYNC_MULTI_RGBD=ON`

**BEFORE YOU CLONE SUBMODULES,** please note that [the Unity simulation](https://github.com/agh-space-systems-rover/unity_sim) repository is included as a submodule in this project, so you will probably have to install **Git LFS** for the submodule to be cloned successfully. If you will want to run the simulation, please follow its README for instructions on how to setup everything properly.

## Getting Started

```bash
ros2 launch kalman kalman.launch.py
# ros2 launch kalman kalman.launch.py unity_sim:=true # to also start the simulation
# ros2 launch kalman kalman.launch.py drivers:=true # to also start the physical drivers
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

## Design Sheet

A **WORK-IN-PROGRESS** summary of the design decisions (to be) made in this project:

### Build Variants

There exist three different build targets implemented as "meta" packages (`rover`, `station`, `full`) that do not contain any code

- `rover` is meant to be installed on the rover where physical hardware is available.
- `station` is to be installed on the ground station unit.
- `full` target should only be built on a developer's PC and it includes the two above packages plus virtual hardware modules managed by the simulator.

There are overlaps between the packages needed by each of the three build variants and this is why their sources are stored as a flat `src` directory of packages instead of dividing them into folders.

###  Source Packaging

The main repository of our project includes all our packages as Git submodules. I can already see how this design forces us to download all packages despite only needing a few of them to be built for the specific target. One good thing about submodules is that we can create tags in the main repository to be able to later return to specific point in time.

###  Launch Files

Each build target exposes launch files specific to the platform.

- `rover` exposes one launch file for every competition type (i.e. autonomous navigation, science, extreme delivery, etc.).
- `station` would only contain a single launch file that starts the station with RF and should work with all operation modes of the rover.
- `full` provides a similar set of launch files, but each one of them relies on the simulator to provide the hardware interface and boot up the ground station without RF communication.

Launch files from previous contests are consistently replaced by the new ones. The old ones end up archived in the Git index.

## Known Issues; TODOs

- sometimes IMU readings are not taken into account by the UKF until Unity process is restarted
- Nav2 randomly stops planning paths and refuses to acknowledge new goals until restart
- Nav2 often fails to communicate with the action server and unnecessarily begins a recovery phase.
- Nav2 sometimes starts spamming "[ActionServer] Aborting handle." indefinitely after goal is aborted.
