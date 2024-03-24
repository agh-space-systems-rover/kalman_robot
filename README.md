# Kalman

Kalman's software stack updated to ROS 2

![](./docs/cover.png)

## Prerequisites

> [!NOTE]
> It is strongly recommended to use [kalman_ws](https://github.com/agh-space-systems-rover/kalman_ws) as the workspace for this project.
> It is a prebuilt ROS 2 workspace that contains all the necessary dependencies for this project.
> It also includes useful shell macros to facilitate the installation of custom Python and APT packages.
> This workspace requires Ubuntu Jammy, but it also fully supports execution and development in a Docker container via Distrobox.

- [Spatio-Temporal Voxel Layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) (Currently not available via rosdep and has to be built manually from source.)
- All Python packages listed in the `requirements.txt` files in the directories of some Colcon packages.
- All APT packages listed in the `apt_packages.txt` files in the directories of some Colcon packages.

## Getting Started

To make development quicker, the launch process of the development setup has been split into two separate launch files. One of them, suffixed `_sim_base`, launches the simulation environment and RViz. The other one, suffixed `_sim_stack`, launches the rest of the software stack, including localization, mapping, and navigation capabilities:
```bash
ros2 launch kalman_bringup urc_autonomy_sim_base.launch.py
# Before running the next command, make sure that the simulation is started and sensor messages are being published.
ros2 launch kalman_bringup urc_autonomy_sim_stack.launch.py
```

In order to run on a physical rover, just one launch file is needed:
```bash
ros2 launch kalman_bringup urc_autonomy_rover.launch.py
```

To launch the ground station, use the following command on a separate machine:
```bash
ros2 launch kalman_bringup urc_autonomy_gs.launch.py
```
For certain features to work, the ground station should be connected to the same network as the rover and/or have our custom radio communication hardware connected to it.

Multiple other launch files are available in the `kalman_bringup` package, including the ones for other competitions and development purposes.

## Packages

Kalman's software stack is composed of multiple packages that are meant to be built and run together:
- `kalman_aruco` - AruCo tags detection using aruco_opencv
- `kalman_bringup` - launch files for the rover and the ground station
- `kalman_description` - Xacro / URDF descriptions + models for the rover
- `kalman_drivers` - drivers for the physical hardware; only to be run separately from the simulation
- `kalman_gazebo` - Ignition Gazebo simulation configs for Kalman
- `kalman_interfaces` - ROS 2 interfaces used by the other `kalman_` packages
- `kalman_mapviz` - configuration and launch files for MapViz
- `kalman_master` - driver for our custom Master device
- `kalman_nav2` - configuration and launch files for Nav2 and related modules; Includes a custom path follower.
- `kalman_robot` - a metapackage that depends on all other `kalman_` packages
- `kalman_rosou` - **WIP** Establishes ROS to ROS communication between the rover and ROS on the ground station. Makes use of the Master device which is the radio communication hub.
- `kalman_slam` - configuration files for robot_localization and RTAB-Map
- `kalman_wheel_controller` - a node that converts Twist messages on `/cmd_vel` and similar topics to the actual wheel state; Also includes safeguards that can limit the acceleration and velocity or stop the rover to adjust wheel rotation.
- `kalman_yolo` - **PRIVATE** YOLO-based object detector and model training toolbox

## Sub-projects

- `point_cloud_utils` - utilities for working with point clouds; Includes ROS wrappers around PCL filters and an obstacle detection node.
- `service_based_nav2_controller` - a `FollowPath` controller plugin for Nav2 that uses a service to compute velocity commands
- `unity_sim` - a Unity-based simulation environment that can seamlessly replace the physical hardware of AGH Space Systems' robots

## Launch Hierarchy

The launch files are organized in a hierarchical manner. The `kalman_bringup` package contains the main launch files that are meant to be the only ones used via `ros2 launch`. `kalman_bringup` includes many other launch files from other `kalman_` packages, which in turn may include even more launch files from other packages:

![](https://quickchart.io/graphviz?graph=digraph{kalman_bringup->kalman_description;kalman_bringup->kalman_drivers->kalman_master;kalman_bringup->kalman_slam;kalman_bringup->kalman_nav2;kalman_bringup->kalman_wheel_controller;kalman_bringup->"...";})

## Data Flow

All `kalman_` packages are designed to work together and exchange data in a complex manner. The following diagram shows the high-level overview of the data flow between top-level modules:

![](https://quickchart.io/graphviz?graph=digraph{kalman_drivers->kalman_slam[label="IMU,%20RGB-D"];kalman_slam->kalman_nav2[label="Odometry"];kalman_nav2->kalman_wheel_controller[label="Twist"];kalman_wheel_controller->kalman_drivers[label="Wheel%20State"];kalman_drivers->kalman_nav2[label="Point%20Clouds"];kalman_supervisor->kalman_nav2[label="Send%20Goal"];kalman_nav2->kalman_supervisor[label="Goal%20Status"];kalman_drivers->kalman_aruco[label="Images"];kalman_aruco->kalman_supervisor[label="Detections"];kalman_gs->kalman_supervisor[label="Objectives"]})

As mentioned in [Launch Hierarchy](#launch-hierarchy), top-level modules may include other modules that are not shown in the diagram. Each module contains a set of nodes that actually perform the data processing and exchange.

## Guidelines

- When committing new code, please follow the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) specification.
- The `scripts` directory should only contain runnable (`chmod +x`) scripts that are meant to be installed as ROS executables.
- The `tools` directory should only contain developer utilities and not ROS-related code.
- Whenever possible, design your C++ nodes as components. See [this tutorial](https://docs.ros.org/en/iron/Tutorials/Intermediate/Writing-a-Composable-Node.html) for more information.
- If a dependency is not available in rosdep, please add it to the `apt_packages.txt` or `requirements.txt` file created next to `package.xml`. Always prefer rosdep over those files.

## Known Issues

- Nav2 (STVL) on random occasions when booting up, starts spamming TF errors and takes up 100% of the CPU. Issue is present both in simulation and on the physical rover.
- Sometimes at random navsat_transform starts publishing bad odometry/gps data which results in map/odom divergence. It happens despite navsat_transform being started after the simulation has applied static TF transforms.
