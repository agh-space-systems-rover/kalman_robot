# kalman_base

This package contains nodes to interface between master device on serial port and ROS2. Additionally, it provides abstract drivers for that communication. It also contains configuration files and launch files for the base.

## Modules

### master_com

Main node responsible for interfacing with the master device. It reads data from serial port and publishes it to ROS2. It also listens for commands from ROS2 and sends them to the Master device. Uses `serial_driver` module.

Every frame received from master is published as a ROS message (*MasterMessage*) on a dynamically
created topic `master_com/master_to_ros/{0x(lowercase hexadecimal frame ID)}`.
Frame data interpretation should be handled by client (or driver).

Every ROS message sent on topic `master_com/ros_to_master` should have *MasterMessage* format.
Message is then encoded as a binary frame and sent out using the serial driver.

### ros_link

This module is responsible for creating a link between two ROS instances, that share the data using serial communication. It requires the same configuration to be set on both sides.

Module listens for messages published by `master_com` and translates it into another topic, that is set in configuration file. It also listens for messages on such topics and sends them to `master_com`.

Has support for services and actions.

### drivers

Drivers, such as `feed_driver`, `drill_driver`, `ueuos_driver`, `wheel_driver` listen on high-level topics and translate them into low-level *MasterMessage* messages. They also may listen for *MasterMessage* messages and translate them into high-level topics.
