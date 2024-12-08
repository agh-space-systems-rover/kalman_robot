# ROS 2 Interfaces

messages, services and actions used by the other `kalman_` packages

Those are the data structures used by APIs in the software stack.

## Messages

- `Arm*`: TODO
- [Drill](./msg/Drill.msg): state of the drilling module
- [Drive](./msg/Drive.msg): teleop control input
- [MasterMessage](./msg/MasterMessage.msg): raw bytes sent to/from the [Master](../kalman_master)
- [WheelState](./msg/WheelState.msg): state of a singular propulsion module (wheel speed + steering angle)
- [WheelStates](./msg/WheelStates.msg): state of all 4 propulsion modules
- [WheelTemperature](./msg/WheelTemperature.msg): temperature of a singular propulsion module (wheel + swivel)
- [WheelTemperatures](./msg/WheelTemperatures.msg): temperature of all 4 propulsion modules

## Services

- [SetFeed](./srv/SetFeed.srv): video feed config update command
- [SetUeuosColor](./srv/SetUeuosColor.srv): change status signalization light (any color)
- [SetUeuosEffect](./srv/SetUeuosEffect.srv): change status signalization light (hardware flashing patterns)
- [SetUeuosState](./srv/SetUeuosState.srv): change status signalization light (predefined colors)
- [SpoofGps](./srv/SpoofGps.srv): construct a fake GPS reading with the given coords and publish it to the system

## Actions

**Autonomous Mission Goals**

- [SupervisorGpsArUcoSearch](./action/SupervisorGpsArUcoSearch.action): search for ArUco markers around a GPS coordinate
- [SupervisorGpsGoal](./action/SupervisorGpsGoal.action): go to a GPS coordinate
- [SupervisorGpsYoloSearch](./action/SupervisorGpsYoloSearch.action): use YOLO neural network to search for an object around a GPS coordinate
- [SupervisorTfGoal](./action/SupervisorTfGoal.action): go to a cartesian coordinate in a frame of reference
