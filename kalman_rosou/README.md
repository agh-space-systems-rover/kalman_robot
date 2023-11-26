# Kalman ROS Over Uart (ROSOU)
Functional copy of [Big Autonomy Translator](https://github.com/agh-space-systems-rover/big_autonomy_translator)

Package used to translate RF frames into ROS messages and the other way around.

# Usage
Start in station mode
```bash
ros2 launch kalman_rosou rosou.launch.py is_station:=True
```

Start in rover mode
```bash
ros2 launch kalman_rosou rosou.launch.py is_station:=False
# or use default setting
ros2 launch kalman_rosou rosou.launch.py
```

# Configuration

To add rover/station publishers edit `config/rosou_config.json`.

To send message from rover to station, add entry in `rover` list.  It will
create subscriber on topic and type given in `topic` and `message_type` fields.

It will serialize part of message specified in `message_part_serialized` at
maximum frequency of `max_frequency`. Then it deserializes the message at
station and publishes it at specified topic.

To send message from station to rover, add entry in `station` list. The flow is
the same as above.

## Technical note
`message_part_serialized` json contains the python code used to access the value of interest
in message `msg`.
So, if you only want to send the `msg.pose.pose.position` portion of `nav_msgs/Odometry`, the
code will run the following:
```python
# Setup
from nav_msgs.msg import Odometry
msg = Odometry()

# Example
to_serialize = eval(config["message_part_serialized"])
## Which will evaluate to
to_serialize = msg.pose.pose.position # right hand taken from json file
```

## Testing
To test this package on linux:
1. Create a virtual serial
2. Make the virtual serial loopback communication
3. Launch kalman_master_driver
4. Launch this package in both station and rover mode
5. Send some data to one of the topics, it should be duplicated

```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0 # Setup virtual serial
# Example output
## N PTY is /dev/pts/6
## N PTY is /dev/pts/7

# Start loopback on first serial
cat /dev/pts/6 | tee /dev/pts/6 | hexdump -C # Loopback with preview

# Launch kalman_master_driver on second serial 
ros2 launch kalman_master_driver master_com.launch.py serial_port:=/dev/pts/7

# Launch kalman_rosou in station mode
ros2 launch kalman_rosou rosou.launch.py is_station:=True

# Launch kalman_rosou in rover mode
ros2 launch kalman_rosou rosou.launch.py is_station:=False

# Start monitoring one of the topics
ros2 topic echo --flow-style /odometry/filtered

# Publish a message on the topic 
ros2 topic pub -1  /odometry/filtered nav_msgs/msg/Odometry '{pose:{pose:{position:{x: 1}}}}'

# In monitoring terminal, you should see the same message duplicated
```