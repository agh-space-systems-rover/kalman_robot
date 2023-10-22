# kalman_base

Note: This package is configured for Python and C++ - for future rewrite in C++.

## Description

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

## Usage

## Nodes

## Uses

## Parameters

## Package structure
