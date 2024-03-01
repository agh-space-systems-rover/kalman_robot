# kalman_base

Note: This package is configured for Python and C++ - for future rewrite in C++.

## Description

Package that provides modules and nodes for handling serial communication with a Kalman rover and is a bridge between Master and ROS.

## Modules

### `lightweight_serial_driver`

#### Description

Module for reading data from serial and writing data to serial.

#### Usage

```python
# Initialize the serial driver
driver = SerialDriver(port_name=port_name, start_byte='<', stop_byte='>', baud_rate=baud_rate, ascii_mode=ascii_mode)

# Read all messages from serial (as list of SerialMsg objects)
driver.tick()
messages: List[SerialMsg] = driver.read_all_msgs()

# Write a message to serial
msg = MasterMessage()

driver.write_msg(msg)
driver.tick()
```

#### SerialMsg

```python
@dataclass
class SerialMsg:
    cmd: UInt8
    argc: UInt8
    argv: List[UInt8]
```

## Nodes

### `master_com`

#### Description

Handles serial communication between Master device and ROS network.

Every frame received from master is published as a ROS message (MasterMessage) on a dynamically
created topic `master_com/master_to_ros/{0x(lowercase hexadecimal frame ID)}`.
Frame data interpretation should be handled by client.

Every ROS message sent on topic `master_com/ros_to_master` should have MasterMessage format.
Message is then encoded as a binary frame and sent out using the serial driver.

Messages should be of type `MasterMessage` and contain the following data:

-   `cmd` - command id
-   `data` - list of arguments

#### Usage

Run as a ROS node.

#### Parameters

-   `/serial_port` - serial port name
-   `/baud_rate` - baud rate
-   `/ascii_mode` - if true, messages are sent as ASCII strings (default: false)

#### Topics

-   `master_com/master_to_ros/{0x(lowercase hexadecimal frame ID)}` - messages received from serial
-   `master_com/ros_to_master` - messages to be sent to serial

## Uses

-   `pyserial`

## Package structure

```
    kalman_base
    ├── CMakeLists.txt              # compiler instructions
    ├── package.xml                 # package metadata
    ├── README.md                   # this file
    ├── kalman_base                 # Python modules
    |   ├── __init__.py             # init file - empty
    |   ├── serial_driver.py        # serial driver helper class
    |   └── master_com.py           # Master-ROS communication node
    └── exec                        # Executables for Python-based nodes
        └── master_com
```
