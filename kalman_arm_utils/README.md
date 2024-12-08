# Arm Utils

utility nodes for the arm - Some scripts are run on groundstation, some on the onboard Raspberry Pi.

## Project Structure

```
kalman_arm_utils
├── scripts                             # Scripts for launching Python nodes
├── kalman_arm_utils                    # Python package with utilities
│   ├── __init__.py
│   └── arm
│       ├── fake_move_server.py         # Fake MoveIt server for capturing trajectories
│       ├── pose_requester.py           # Script for requesting predefined poses
│       ├── trajectory_requester.py     # Script for requesting predefined trajectories
│       └── servo_param_setter_node.py  # Script for setting MoveIt servo parameters from topics
└── src                                 # Source files for C++ utilities
    ├── arm                             # C++ utilities for arm
    │   ├── arm_state_publisher.cpp     # Node for publishing arm state in custom format
    └── gs                              # C++ utilities for groundstation
        ├── arm_state_republisher.cpp   # Node for republishing arm state from custom format
        ├── gripper_republisher.cpp     # Node for republishing gripper command from inputs
        └── joint_republisher.cpp       # Node for republishing joint command from inputs
```
