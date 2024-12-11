# Arm Controller

a set of nodes that use ROS2 Control and CAN to interface with joints and MoveIt controllers

## Description

The main element is the hardware controller, which is used by ROS2 control to interface with arm's joints using CAN FS protocol.

Additionally, there are some extra ROS nodes in the `scripts` directory. Those are used during development.

## Kalman's Arm Description

Here's a diagram of data flow between the arm's subsystems. Our custom nodes are colored in green:

![](docs/arm_control.drawio.svg)

(Excuse Polish language in the diagram.)
