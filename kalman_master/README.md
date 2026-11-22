# Master

driver for our custom "Master" device

Communicates with the Master over serial and exposes its functionality as a ROS 2 API, using messages and services.


> [!NOTE]
> Master is a device that controls the rover's subsystems.
> It communicates over a serial port using a custom protocol.
> The protocol is based on a frame structure, where each frame has a unique ID and a payload.
> The ID determines the function of the frame, and the payload is the data that the frame carries.
> The frames forwarded by Master can control of the rover's wheels, drill, and other actuators, as well as read data from sensors like the IMU and encoders.
> Additionally, the Master can send data over the radio to a remote adapter connected to the ground station.


## Nodes

The serial transport (`master_com`), the radio link (`ros_link`) and `master_loopback` live in [`kalman_tunnel`](../kalman_tunnel). `launch/master.launch.py` only starts the drivers below, so `kalman_tunnel`'s `tunnel.launch.py` must run alongside it. `kalman_hardware`'s `hardware.launch.py master:=...` starts both.

#### Drivers

Nodes such as `feed_driver`, `drill_driver`, `ueuos_driver`, `wheel_driver`, expose a high-level API for different functionalities of the Master. They translate the API into the low-level `MasterMessage` format sent to and received on the `master_com`'s topics.

#### arm_twist_driver

Interprets linear and angular velocity inputs and translates them into Master's arm control frames.
Used to steer the arm using a 3D Space Mouse.
