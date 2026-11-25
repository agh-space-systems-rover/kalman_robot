# Drivers

ROS drivers for the subsystems of our custom "Master" device

Exposes the Master's functionality as a ROS 2 API, using messages and services.


> [!NOTE]
> Master is a device that controls the rover's subsystems.
> It communicates over a serial port using a custom protocol.
> The protocol is based on a frame structure, where each frame has a unique ID and a payload.
> The ID determines the function of the frame, and the payload is the data that the frame carries.
> The frames forwarded by Master can control of the rover's wheels, drill, and other actuators, as well as read data from sensors like the IMU and encoders.
> Additionally, the Master can send data over the radio to a remote adapter connected to the ground station.


## The Master bus

The drivers never touch the serial port. They exchange [`MasterMessage`](../kalman_interfaces/msg/MasterMessage.msg) frames with `master_com` from [`kalman_tunnel`](../kalman_tunnel) on these topics:

- `master_com/ros_to_master`: frames to send to the Master
- `master_com/master_to_ros/x<id>`: frames received from the Master, one topic per frame ID (lowercase hex, e.g. `x5a`)

Build these names with `kalman_drivers.master_topics` (Python) or `kalman_drivers/master_topics.hpp` (C++) instead of by hand.

Those topics and `kalman_interfaces` are the whole contract, so this package does not depend on `kalman_tunnel`. `launch/drivers.launch.py` only starts the drivers, so `kalman_tunnel`'s `tunnel.launch.py` must run alongside it. `kalman_hardware`'s `hardware.launch.py master:=...` starts both.

## Nodes

#### Drivers

Nodes such as `feed_driver`, `ueuos_driver`, `wheel_driver`, expose a high-level API for different functionalities of the Master. They translate the API into the low-level `MasterMessage` format sent to and received on the `master_com`'s topics.

#### arm_twist_driver

Interprets linear and angular velocity inputs and translates them into Master's arm control frames.
Used to steer the arm using a 3D Space Mouse.

#### tunnel

Ground-station node (executable `tunnel_client`) that asks the rover over the radio to reboot its PC or to start the ARCh stack/goal. Each call queues `GS_TO_PC` frames on `master_com/ros_to_master`. Services (`std_srvs/Empty`):

- `/tunnel/reboot_pc`
- `/tunnel/arch_launch_stack`
- `/tunnel/arch_send_goal`

#### Dropped drivers

The science, drill and camera_2dof features are no longer used, so `drivers.launch.py` does not start `drill_driver`, `storage_driver`, `sand_storage_driver`, `ph_driver` or `camera_2dof_driver` in any mode. They and their tools (`ph_calib`, `ph_mock`, `storage_mock`) are still installed and can be started by hand with `ros2 run kalman_drivers <executable>`.
