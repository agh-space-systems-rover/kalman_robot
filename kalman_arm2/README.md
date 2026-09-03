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

Trigger panel location mission: `ros2 action send_goal /arm/move_to_panel_pose kalman_interfaces/action/MoveToPanelPose '{behavior_tree: demo, target_pose: {position: {x: 0.0, y: 0.0, z: 0.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'`


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
ros2 action send_goal /arm/move_to_panel_pose kalman_interfaces/action/MoveToPanelPose   '{behavior_tree: demo, target_pose: {position: {x: -0.07517217, y: 0.07195299569680749, z: 0.09098195987914491}, orientation: {x: 0.0, y: 0.0, z: -0.383, w: 0.924}}}'
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

**3. Fake rectified detections** (3 clickable buttons on 450×650 image from bag):

```bash
ros2 run kalman_arm2 fake_yolo_publisher
```

**4. If homography missing**, publish identity matrix so `sendXY == imageXY`:

```bash
ros2 run kalman_arm2 publish_identity_homography
```

**5. In GS panel:** click bbox center, then:

- **SEND Approach** runs the `approach` behavior tree.
- **SEND Interact** runs the tree named by the raw detection `class_id`.
- The paired **Abort** button cancels the active action.
- The text field below each button pair displays action feedback and result.

The panel sends goals directly to `/arm/move_to_panel_pose` with target z fixed at 0.05 m.

CLI equivalent:

```bash
ros2 action send_goal /arm/move_to_panel_pose kalman_interfaces/action/MoveToPanelPose \
"{behavior_tree: approach, target_pose: {position: {x: 0.1, y: 0.2, z: 0.05}, orientation: {w: 1.0}}}" \
--feedback
```

### Panel topics (kalman_gs)

| Topic | Type | Role |
|-------|------|------|
| `/arm/panel/image_rectified` | `sensor_msgs/Image` | camera |
| `/arm/panel/detections_rectified` | `vision_msgs/Detection2DArray` | bbox overlay in rectified-image pixels |
| `/arm/panel/homography` | `std_msgs/Float64MultiArray` | row-major 3×3 rectified pixel → panel meter transform |

### Panel marker ID calibration

Panel marker geometry and allowed IDs come from `config/panel_layout.yaml`. Runtime assignments reset to YAML values when the node restarts.

Automatically infer IDs after five consecutive matching detection frames:

```bash
ros2 action send_goal /arm/calibrate_panel_marker_ids \
  kalman_interfaces/action/CalibratePanelMarkerIds \
  "{required_confirmations: 5, timeout_seconds: 10.0}" --feedback
```

Set an assignment manually. Names must cover every YAML marker slot and IDs must be unique members of `allowed_marker_ids`:

```bash
ros2 service call /arm/set_panel_marker_ids \
  kalman_interfaces/srv/SetPanelMarkerIds \
  "{marker_names: [top_left, top_right, bottom_left], marker_ids: [13, 14, 15]}"
```
