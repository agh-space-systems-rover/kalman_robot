# kalman_tools

Auxiliary desktop and operator tools for the Kalman robot stack.

## Master Frame Panel

Streamlit app for sending and receiving Master protocol frames through rosbridge.

- **Send:** publish `kalman_interfaces/msg/MasterMessage` on `master_com/ros_to_master`
- **Receive:** subscribe to `master_com/master_to_ros/xNN` (one topic per frame ID)

### Prerequisites

- Built workspace with `kalman_tools`, `kalman_interfaces`, and `rosbridge_server`
- Python dependencies:

```bash
pip install -r src/kalman_robot/kalman_tools/requirements.txt
```

- A running Master bridge in the ROS graph:
  - `master_com` for hardware, or
  - `master_loopback` for local testing

### Launch

Standalone (starts rosbridge on port 3001 and opens Streamlit on port 8501):

```bash
ros2 launch kalman_tools master_frame_panel.launch.py
```

Attach to an existing Ground Station rosbridge:

```bash
ros2 launch kalman_tools master_frame_panel.launch.py start_rosbridge:=false rosbridge_port:=9065
```

Run Streamlit only (rosbridge must already be reachable):

```bash
ros2 run kalman_tools master_frame_panel
```

### Usage

Operator how-to (launch, UI, troubleshooting): [`docs/kalman_tools.md`](../../../docs/kalman_tools.md) at the workspace root.

### Manual test checklist

1. Start `master_loopback`, launch the panel, send `AUTONOMY_SWITCH` with payload `1`, confirm receive on topic suffix `x20` after selecting that ID on the right.
2. Start spam at 1 s interval, verify repeated sends, then stop per-row and with **Emergency Intercept: Cease All Spams**.
3. Select multiple receive frame IDs and confirm the log topics update.
4. Add a payload filter (index, operator, byte) and verify only matching messages appear.
5. With GS running, use `start_rosbridge:=false rosbridge_port:=9065`.

### Adding another tool

1. Add a module under `kalman_tools/apps/`.
2. Add a launch file under `launch/<tool>.launch.py`.
3. Optionally add a wrapper script under `scripts/`.
4. Document usage in this README.
