# Wheel Controllers

High level API for the wheel control system.

Provides an abstraction layer over low-level wheel hardware/simulation drivers.

While the hardware driver or the simulation allow to set the speed and rotation of each wheel independently, the high-level API can interpret either the twist of the robot or human driving input.

As an extra, there are two headless input controllers that can be used to control the robot without access to the ground station interface.

## Controllers

#### [Twist](./kalman_wheels/twist_controller_node.py)

Interprets linear and angular velocities of the robot in relation to its centroid.

Includes a safeguard system that constrains acceleration and deceleration, and additionally stops the robot to adjust when target twist changes too abruptly.

- Message: [`Twist`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html)
- Used by: the autonomous system
- Source: [`twist_controller_node.py`](./kalman_wheels/twist_controller_node.py)

#### [Drive](./kalman_wheels/drive_controller_node.py)

Interprets human driving input:

- movement speed
- turn radius
- body angle offset
- in-place rotation

In this case, acceleration is not constrained to allow the human operator to execute more dangerous maneuvers.

- Message: [`Drive`](../kalman_interfaces/msg/Drive.msg) 
- Used by: human input subsystems
- Source: [`drive_controller_node.py`](./kalman_wheels/drive_controller_node.py)

## Headless Input

#### [Gamepad](./kalman_wheels/gamepad_driving_node.py)

Handles gamepad input and publishes `Drive` messages. This node is a headless alternative to gamepad handling in `kalman_gs`. Only one gamepad can be used.

#### [Arduino](./kalman_wheels/arduino_driving_node.py)

Handles inputs from our custom Arduino controller and publishes `Drive` messages. This node is run along with `kalman_ws` as it does not support the Arduino controller by itself.
