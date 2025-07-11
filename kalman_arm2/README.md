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
