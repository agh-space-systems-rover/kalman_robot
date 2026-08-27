# kalman_arm_autonomy_panel

Dev launch and rosbag fixture for the **Arm Autonomy** panel in `kalman_gs`.

## Quick start

```bash
# build once after pulling this package
colcon build --packages-select kalman_arm_autonomy_panel --symlink-install
source install/setup.bash

ros2 launch kalman_arm_autonomy_panel arm_autonomy_panel_dev.launch.py
```

Open the ground station in a browser, add the **Arm Autonomy** panel.

This launch file:

1. Plays `rosbag2_2026_08_26-00_51_05` in a loop
2. Remaps `/arm/panel/pixel_to_panel` → `/arm/panel/homography`
3. Starts `kalman_gs` (rosbridge on port 9065 + frontend)

## Rosbag contents

| Topic in bag | Type | Panel topic |
|--------------|------|-------------|
| `/arm/panel/image_rectified` | `sensor_msgs/Image` | same |
| `/arm/panel/pixel_to_panel` | `std_msgs/Float64MultiArray` | `/arm/panel/homography` (via remap) |

`/yolo_detections` is **not** in the bag. Publish fake detections manually:

```bash
ros2 topic pub --rate 2 /yolo_detections vision_msgs/msg/Detection2DArray \
"{detections: [{bbox: {center: {position: {x: 320.0, y: 240.0}}, size_x: 80.0, size_y: 60.0}, \
results: [{hypothesis: {class_id: 'button', score: 0.9}}]}]}"
```

Adjust bbox coordinates to match the image resolution from the bag.

## Manual playback (without this launch)

```bash
ros2 bag play \
  src/kalman_robot/kalman_arm_autonomy_panel/rosbag2_2026_08_26-00_51_05 \
  --loop \
  --remap /arm/panel/pixel_to_panel:=/arm/panel/homography

ros2 launch kalman_gs gs.launch.py
```

## Panel topics (kalman_gs)

- `/arm/panel/image_rectified` — `sensor_msgs/Image`
- `/yolo_detections` — `vision_msgs/Detection2DArray`
- `/arm/panel/homography` — `std_msgs/Float64MultiArray` (3×3 row-major)

SEND button is a stub; ROS action client will be added later.
