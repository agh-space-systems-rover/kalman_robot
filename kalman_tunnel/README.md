# Tunnel

serial and radio transport between ROS and the Master device

Carries raw frames between ROS and the Master over serial, and uses the Master's radio to connect the ground station and the rover. The device drivers that build on top of it live in [`kalman_master`](../kalman_master).


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

```bash
colcon test --packages-select kalman_tunnel --ctest-args -R test_ros_link_loopback
```

For manual experiments, [`launch/test_ros_link.launch.py`](./launch/test_ros_link.launch.py) starts the same setup with 50% frame loss.
