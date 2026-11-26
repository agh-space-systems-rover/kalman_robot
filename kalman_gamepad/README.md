# Gamepad

Gamepad control of the wheels, arm and drill.

The node reads gamepads connected to the computer it runs on directly from `/dev/input/js*`, so driving works without the ground station. Gamepads are detected when plugged in. The ground station only displays the connected gamepads and changes their modes.

Gamepads must use the standard Linux layout, which means XInput mode for Logitech gamepads. Other joysticks are reported as unsupported and ignored. Our custom Arduino controller and the SpaceMouse are only reported to the station, because they are handled by `kalman_wheels` and `kalman_spacenav`.

## Modes

Each gamepad has one of the modes:

- none - the default, the gamepad does nothing
- wheels
- arm
- drill

Select (or the Logitech button) cycles through the modes in this order. Start switches to wheels. The station can set any mode using the `/station/gamepads/set_mode` service.

When multiple gamepads are in the same mode, their inputs are averaged.

## Controls

#### Wheels

- right trigger / left trigger - drive forward / backward
- left stick X - turn
- right stick X - body angle
- left or right shoulder - rotate in place, triggers set the speed
- Y / X - increase / decrease the speed limit
- left / right stick press - horn on the station

#### Arm

- left stick X, left stick Y, right stick Y, right stick X - joints 1-4
- D-pad up / down - joint 5
- right trigger / left trigger - joint 6
- Y / A - gripper
- right / left shoulder - move the joint lock focus on the station
- X / B - fastclick 0 / 255
- D-pad left - toggle the music on the station

Joints locked on the station are not moved.

#### Drill

- left stick Y or D-pad up / down - bridge B
- left stick X or D-pad right / left - bridge C
- B - stop
- X - home
- Y - open tubes on both sites
- A - clean the drill
- left stick press, left trigger, left shoulder - drill, open tubes, close tubes on site 1
- right stick press, right trigger, right shoulder - drill, open tubes, close tubes on site 2

## Interface

- `/station/gamepads/state` (`kalman_interfaces/GamepadStates`, transient local) - connected devices and their modes; Published on every change and every second.
- `/station/gamepads/set_mode` (`kalman_interfaces/SetGamepadMode`) - changes the mode of a gamepad
- `/station/gamepads/sound` (`std_msgs/String`) - sounds to play on the station: `horn_left`, `horn_right`, `interstellar`
- `/station/arm/joint_locks` (`std_msgs/UInt8`, transient local, subscribed) - bitmask of joints locked on the station; Bit 0 is joint 1.
- `/station/arm/joint_lock_focus` (`std_msgs/UInt8`, transient local) - joint highlighted on the station, from 1 to 6
- `/drive` (`kalman_interfaces/Drive`)
- `/station/arm/fk/command` (`kalman_interfaces/ArmFkCommand`) - only published when a gamepad is in arm mode
- `/fastclick` (`std_msgs/UInt8`)
- `/science/drill/b`, `/science/drill/c` (`std_msgs/Int8`), `/science/drill/state` (`std_msgs/UInt8`) - transient local, so that rosbridge receives them even if the station also published there; Only published on change.
