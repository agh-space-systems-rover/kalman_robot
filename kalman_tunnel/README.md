# Tunnel

serial and radio transport between ROS and the Master device

Carries raw frames between ROS and the Master over serial, and uses the Master's radio to connect the ground station and the rover. The device drivers that build on top of it live in [`kalman_drivers`](../kalman_drivers).


## Usage

```bash
ros2 launch kalman_tunnel tunnel.launch.py mode:=gs  # or pc, arm
```

| Mode | `master_com` baud rate | `ros_link` sides |
|---|---|---|
| `gs` | 38400 | `ros_link_gs_to_pc`, `ros_link_gs_to_arm` (station) |
| `pc` | 115200 | `ros_link_pc_to_gs` (rover) |
| `arm` | 2000000 | `ros_link_arm_to_gs` (rover) |

`port:=/dev/tty...` sets the serial port. By default the Master is found automatically by its USB model.

The ROS interface into the rover is the `MasterMessage` bus: publish frames on `master_com/ros_to_master`; frames from the Master arrive on `master_com/master_to_ros/x<frame ID in hex>`.


## Nodes

#### master_com

Main node, responsible for interfacing with the device. It reads and writes data from a serial port, and exposes it to ROS 2. The node is split into two parts, the ROS node [`master_com_node.py`](./kalman_tunnel/master_com_node.py) and [`serial_driver.py`](./kalman_tunnel/serial_driver.py) which encapsulates the serial communication logic and the Master's protocol.

Every frame received from the Master on a serial port is published as a ROS message [`MasterMessage`](../kalman_interfaces/msg/MasterMessage.msg) on a dynamically created topic `master_com/master_to_ros/{0x(lowercase hexadecimal frame ID)}`.

Here each data frame sent or received from the Master is identified by a unique ID. The ID is a 1-byte number which determines the function of that frame, for example, a frame that controls the speed of the robot's wheels will have a different ID than a frame that controls the drill. `MasterMessage` is basically a byte array, but it stores the ID as a separate field for clarity.

In order to send data to the Master, `MasterMessage`s should be published to `master_com/ros_to_master`. `master_com` will then forward the received frames to the serial port.

#### ros_link

This node leverages the Master's ability to send data over the radio to connect two independent ROS instances (here ground station and the rover) using RF comms.

It reads a [YAML config](./config/pc_ros_link.yaml) to determine which topics/services/actions to forward to the other end. The config specifies two sides of communication and must be replicated on both ends for the link to work.

#### master_loopback

Alternative `master_com` implementation that allows to test nodes that use its topics. Every message sent to `master_com/ros_to_master` will get echoed back to `master_com/master_to_ros/{0xID}`. This is useful for testing the behavior of the nodes that use the Master's low-level API without having the Master connected. `master_loopback` also allows to simulate various message loss rates and delays to test the robustness of the system. This came in handy when debugging `ros_link`!


## Testing

[`test/test_ros_link_loopback.py`](./test/test_ros_link_loopback.py) runs both sides of `ros_link` through `master_loopback` and checks that topics, services and actions cross the link. No hardware is needed:

[`test/test_tunnel_launch.py`](./test/test_tunnel_launch.py) starts `tunnel.launch.py` in each mode with `master_com` on a virtual serial port, and checks the started nodes, their parameters and frames crossing the port.

```bash
colcon test --packages-select kalman_tunnel
```

For manual experiments, [`launch/test_ros_link.launch.py`](./launch/test_ros_link.launch.py) starts the same setup with 50% frame loss.
