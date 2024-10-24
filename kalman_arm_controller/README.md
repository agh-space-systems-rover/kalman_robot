# kalman_arm_controller

ROS package for controlling Kalman's arm. It includes CAN related library to interface with joints as well as MoveIt controllers.

## Description

The main element of this package is hardware controller used by ROS2 control to interface with the arm joints. For interfacing it uses CAN FD protocol.

Additionally in this pacakge there are some scripts used for testing and providing extra functionalities.

## Kalman's Arm Description

![](../docs/arm_control.drawio.svg)

Above you can find a simple diagram showing data flow in arm. Green blocks represent code that is written for custom usage of ROS2 Control.
