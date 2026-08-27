# kalman_arm_autonomy_panel

Dev launch and rosbag fixture for the **Arm Autonomy** panel in `kalman_gs`.

## Quick start

```bash
colcon build --packages-select kalman_arm_autonomy_panel --symlink-install
source install/setup.bash

ros2 launch kalman_arm_autonomy_panel arm_autonomy_panel_dev.launch.py
```

Open the ground station in a browser, add the **Arm Autonomy** panel.

## Test bbox click → sendXY → SEND (full flow)

Terminal layout (after `source install/setup.bash`):

**1. Bag + homography remap** (or use dev launch above):

```bash
ros2 bag play \
  src/kalman_robot/kalman_arm_autonomy_panel/rosbag2_2026_08_26-00_51_05 \
  --loop \
  --remap /arm/panel/pixel_to_panel:=/arm/panel/homography
```

**2. Ground station** (if not using dev launch):

```bash
ros2 launch kalman_gs gs.launch.py
```

**3. Fake YOLO detections** (3 clickable buttons on 450×650 image from bag):

```bash
ros2 run kalman_arm_autonomy_panel fake_yolo_publisher
```

Optional args: `--rate 5`, `--topic /yolo_detections`.

**4. If homography missing** (bag not remapped), publish identity matrix so `sendXY == imageXY`:

```bash
ros2 run kalman_arm_autonomy_panel publish_identity_homography
```

**5. In GS panel:**

- Open DevTools console → filter `[arm-autonomy]`
- Click circle on a bbox center → status shows `image XY` and `send XY`
- Click **SEND** → alert stub with panel coordinates

### One-liner alternative (single static bbox)

```bash
ros2 topic pub --rate 2 /yolo_detections vision_msgs/msg/Detection2DArray \
"{header: {frame_id: panel_camera}, detections: [{bbox: {center: {position: {x: 225.0, y: 320.0}}, size_x: 90.0, size_y: 70.0}, results: [{hypothesis: {class_id: 'button_a', score: 0.95}}]}]}"
```

Coordinates are pixel centers on `/arm/panel/image_rectified` (bag default: **450×650**, `bgr8`).

## Dev launch

`arm_autonomy_panel_dev.launch.py` runs bag play (with homography remap) + `kalman_gs`.

Does **not** start fake YOLO — run `fake_yolo_publisher` in a separate terminal.

## Rosbag contents

| Topic in bag | Type | Panel topic |
|--------------|------|-------------|
| `/arm/panel/image_rectified` | `sensor_msgs/Image` | same |
| `/arm/panel/pixel_to_panel` | `std_msgs/Float64MultiArray` | `/arm/panel/homography` (via remap) |

## Verify SEND output

In a separate terminal:

```bash
ros2 topic echo /arm/panel/target
```

After clicking **SEND** in the panel you should see `geometry_msgs/PointStamped`:

- `point.x`, `point.y` — panel coordinates (`sendXY`)
- `header.frame_id` — detection label (e.g. `button_a (95%)`)

Full click + image coords are also logged in the browser console (`[arm-autonomy] published panel target`).

## Panel topics (kalman_gs)

- `/arm/panel/image_rectified` — `sensor_msgs/Image`
- `/yolo_detections` — `vision_msgs/Detection2DArray`
- `/arm/panel/homography` — `std_msgs/Float64MultiArray` (3×3 row-major)
- `/arm/panel/target` — `geometry_msgs/PointStamped` (output on SEND)

Arm action server for `/arm/panel/target` is not implemented yet — subscribe with `ros2 topic echo` or your own node.
