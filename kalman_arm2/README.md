- republish joint states from /arm/joints/current_pos to /joint_states to see the arm in RViz
- node that converts EE /arm/ik/target_twist to /arm/joints/target_vel using IK via KDL
- gamepad control node that translates /joy to /arm/ik/target_twist for intuitive arm control

## Gamepad Control Scheme
- Left stick: Linear movement in Y/Z plane (relative to end-effector)
- Right stick: Pitch and yaw rotation
- Triggers: Linear movement forward/backward (X axis)
- Shoulder buttons: Roll rotation (digital input)
- A button (South): Close jaw (negative velocity)
- Y button (North): Open jaw (positive velocity)

Launch with gamepad control: `ros2 launch kalman_arm2 arm2.launch.py enable_gamepad:=true`

Trigger panel location mission: `ros2 action send_goal /arm/move_to_panel_pose kalman_interfaces/action/MoveToPanelPose '{target_pose: {position: {x: 0.0, y: 0.0, z: 0.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'`


## Notes for future
To make the arm avoid twisting joint 5, you can use the following, or make the gain selectively bigger for joint 4:
```python
            DeclareLaunchArgument(
                "ik_joint_centering_gain",
                default_value="1.5",
                description="Nullspace gain that pulls joints toward preferred positions.",
            ),
```
