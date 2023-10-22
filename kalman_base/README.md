# kalman_base

Note: This package is configured for Python and C++ - for future rewrite in C++.

## Description

Package that provides modules and nodes for handling serial communication with a Kalman rover and is a bridge between UART and ROS.

## Modules

### `lightweight_serial_driver`

#### Description

Module for reading data from serial and writing data to serial.

#### Usage

```python
# Initialize the serial driver
driver = LightweightSerialDriver(
            _port_name=port_name, _START_BYTE='<', _STOP_BYTE='>', _BAUD=baud_rate,ascii_mode=ascii_mode)

# Read all messages from serial (as list of UartMsg objects)
driver.tick()
messages: List[UartMsg] = driver.read_all_msgs()

# Write a message to serial
msg = UInt8MultiArray()

driver.write_msg(msg)
driver.tick()
```

#### UartMsg

```python
@dataclass
class UartMsg:
    cmd: UInt8
    argc: UInt8
    argv: List[UInt8]
```

## Nodes

### `uart_ros_bridge`

#### Description

Simple node handling serial communication - publishing received UART messages to ROS topics (`/kalman_rover/uart2ros/{id}`) and sending UART messages from ROS topic (`/kalman_rover/ros2uart`). Messages should be of type `UInt8MultiArray` and contain the following data:

-   `cmd` - command id
-   `argc` - number of arguments
-   `argv` - list of arguments

#### Usage

Run as a ROS node.

#### Parameters

-   `/serial_port` - serial port name
-   `/baud_rate` - baud rate
-   `/ascii_mode` - if true, messages are sent as ASCII strings (default: false)

#### Topics

-   `/kalman_rover/uart2ros/{id}` - UART messages received from serial
-   `/kalman_rover/ros2uart` - UART messages to be sent to serial

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
    |   └── lightweight_serial_driver.py
    ├── scripts                     # Python nodes
    │   └── uart_ros_bridge.py
    ├── src                         # C++ source code
    |   └── main.cpp                # main file - empty for now
    └── include                     # C++ headers
        └── kalman_base             # C++ headers
            └── main.hpp            # main header - empty for now
```
