# Kalman ROS Over Uart (ROSOU)
Functional copy of [Big Autonomy Translator](https://github.com/agh-space-systems-rover/big_autonomy_translator)

Package used to translate RF frames into ROS messages and the other way around.

# Configuration

To add rover/station publishers edit `cfg/rosou_config.json`.

To send message from rover to station, add entry in `rover` list.  It will
create subscriber on topic and type given in `topic` and `message_type` fields.
It will serialize part of message specified in `message_part_serialized` at
maximum frequency of `max_frequency`. Then it deserializes the message at
station and publishes it at specified topic.

To send message from station to rover, add entry in `station` list. The flow is
the same as above.
