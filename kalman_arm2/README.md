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


## Notes
For center knob:
```bash
ros2 action send_goal /arm/move_to_panel_pose kalman_interfaces/action/MoveToPanelPose   '{target_pose: {position: {x: -0.07517217, y: 0.07195299569680749, z: 0.09098195987914491}, orientation: {x: 0.0, y: 0.0, z: -0.383, w: 0.924}}}'
```

## Arm Autonomy GS panel (dev)

Dev launch and rosbag fixture for the **Arm Autonomy** panel in `kalman_gs`.

### Quick start

```bash
colcon build --packages-select kalman_arm2 kalman_gs --symlink-install
source install/setup.bash

ros2 launch kalman_arm2 arm_autonomy_panel_dev.launch.py
```

Open the ground station in a browser, add the **Arm Autonomy** panel.

### Test bbox click → sendXY → SEND (full flow)

**1. Bag + homography remap** (or use dev launch above):

```bash
ros2 bag play \
  src/kalman_robot/kalman_arm2/rosbag2_2026_08_26-00_51_05 \
  --loop \
  --remap /arm/panel/pixel_to_panel:=/arm/panel/homography
```

**2. Ground station** (if not using dev launch):

```bash
ros2 launch kalman_gs gs.launch.py
```

**3. Fake YOLO detections** (3 clickable buttons on 450×650 image from bag):

```bash
ros2 run kalman_arm2 fake_yolo_publisher
```

**4. If homography missing**, publish identity matrix so `sendXY == imageXY`:

```bash
ros2 run kalman_arm2 publish_identity_homography
```

**5. In GS panel:** click bbox center → **SEND** → verify with:

```bash
ros2 topic echo /arm/panel/target
```

**Optional — fake SEND without GS** (same `PoseStamped` format as panel):

```bash
ros2 run kalman_arm2 publish_fake_panel_target
# or periodic:
ros2 run kalman_arm2 publish_fake_panel_target --rate 1 --x 225 --y 320 --frame-id button_a
```

CLI one-liner:

```bash
ros2 topic pub --once /arm/panel/target geometry_msgs/msg/PoseStamped \
"{header: {frame_id: button_a}, pose: {position: {x: 225.0, y: 320.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### Panel topics (kalman_gs)

| Topic | Type | Role |
|-------|------|------|
| `/arm/panel/image_rectified` | `sensor_msgs/Image` | camera |
| `/yolo_detections` | `vision_msgs/Detection2DArray` | bbox overlay |
| `/arm/panel/homography` | `std_msgs/Float64MultiArray` | image → panel |
| `/arm/panel/target` | `geometry_msgs/PoseStamped` | SEND output |
