# Master

driver for our custom "Master" device

Communicates with the Master over serial and exposes its functionality as a ROS 2 API, using messages and services.

## Nodes

#### master_com

Main node, responsible for interfacing with the device. It reads and writes data from a serial port, and exposes it to ROS 2. The node is split into two parts, the ROS node [`master_com_node.py`](./kalman_master/master_com_node.py) and [`serial_driver.py`](./kalman_master/serial_driver.py) which encapsulates the serial communication logic and the Master's protocol.

Every frame received from the Master on a serial port is published as a ROS message [`MasterMessage`](../kalman_interfaces/msg/MasterMessage.msg) on a dynamically created topic `master_com/master_to_ros/{0x(lowercase hexadecimal frame ID)}`.

Here each data frame sent or received from the Master is identified by a unique ID. The ID is a 1-byte number which determines the function of that frame, for example, a frame that controls the speed of the robot's wheels will have a different ID than a frame that controls the drill. `MasterMessage` is basically a byte array, but it stores the ID as a separate field for clarity.

In order to send data to the Master, `MasterMessage`s should be published to `master_com/ros_to_master`. `master_com` will then forward the received frames to the serial port.

#### Other Drivers

Later, other nodes, such as `feed_driver`, `drill_driver`, `ueuos_driver`, `wheel_driver`, expose a high-level API for different functionalities of the Master. They translate the API into the low-level `MasterMessage` format sent to and received on the `master_com`'s topics.

#### ros_link

This node leverages the Master's ability to send data over the radio to connect two independent ROS instances (here ground station and the rover) using RF comms.

It reads a [YAMl config](./config/pc_ros_link.yaml) to determine which topics/services/actions to forward to the other end. The config specifies two sides of communication and must be replicated on both ends for the link to work.

#### master_loopback

Alternative `master_com` implementation that allows to test nodes that use its topics. Every message sent to `master_com/ros_to_master` will get echoed back to `master_com/master_to_ros/{0xID}`. This is useful for testing the behavior of the nodes that use the Master's low-level API without having the Master connected. `master_loopback` also allows to simulate various message loss rates and delays to test the robustness of the system. This came in handy when debugging `ros_link`!

#### arm_twist_driver

Interprets linear and angular velocity inputs and translates them into Master's arm control frames.
Used to steer the arm using a 3D Space Mouse.
