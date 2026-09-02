# Master Frame Panel

Streamlit operator UI for Master protocol frames. It talks to ROS through rosbridge, not as a native ROS node.

- **Send:** publish `kalman_interfaces/msg/MasterMessage` on `master_com/ros_to_master`
- **Receive:** subscribe to `master_com/master_to_ros/xNN` (one topic per frame ID)

Run this from the **kalman_ws distrobox**, with the workspace sourced.

## Prerequisites

1. Build after pulling changes:

```bash
build kalman_tools
```

2. Python deps (once per distrobox):

```bash
pip install -r src/kalman_robot/kalman_tools/requirements.txt
```

3. A Master bridge in the ROS graph:

- `ros2 run kalman_master master_loopback` for local testing
- `ros2 run kalman_master master_com` when hardware Master is connected

Without one of those, send still publishes, but nothing echoes back into the receive log.

## Launch (local test)

Two terminals.

**Terminal 1 — loopback**

```bash
ros2 run kalman_master master_loopback
```

Keep it running. Loopback echoes `master_com/ros_to_master` to `master_com/master_to_ros/{hex cmd}` (for example `x20`). Default `loss_rate` is `0.5`, so some frames are dropped on purpose.

**Terminal 2 — panel**

```bash
ros2 launch kalman_tools master_frame_panel.launch.py
```

This starts:

- `rosbridge_websocket` on `ws://localhost:3001`
- Streamlit on `http://localhost:8501`

Open the dashboard:

```text
http://localhost:8501
```

Wait until the sidebar shows **Node Status: Online** before sending.

### Launch arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `start_rosbridge` | `true` | Start a local `rosbridge_websocket` |
| `rosbridge_host` | `localhost` | Host the Streamlit app connects to |
| `rosbridge_port` | `3001` | Rosbridge WebSocket port (GS uses `9065`) |
| `streamlit_port` | `8501` | HTTP port for the panel |

Attach to an existing Ground Station rosbridge:

```bash
ros2 launch kalman_tools master_frame_panel.launch.py start_rosbridge:=false rosbridge_port:=9065
```

Streamlit only (rosbridge already reachable):

```bash
ros2 run kalman_tools master_frame_panel
```

Override host/port via env if you run Streamlit yourself:

```bash
KALMAN_TOOLS_ROSBRIDGE_HOST=localhost KALMAN_TOOLS_ROSBRIDGE_PORT=3001 streamlit run $(python3 -c "from importlib.util import find_spec; print(find_spec('kalman_tools.apps.master_frame_panel').origin)")
```

## Use the panel

Layout:

- **Sidebar:** connection (host/port, read-only) and telemetry refresh rate
- **Left — Transmit Commands:** send rows
- **Right — Diagnostic Telemetry Logs:** receive filter + log

### Send (left)

Each bordered row is one frame recipe.

1. Optional **Label / Operational Alias**
2. Pick a **Catalog Blueprint Lookup** (names from `MasterMessage` constants) or type a **Command Register Identifier** (`AUTONOMY_SWITCH`, `0x20`, or decimal)
3. **Data Payload Array (uint8):** empty, decimals, or hex, separated by space/comma/semicolon. Example: `1` or `0x01, 0x00, 255`
4. Check **Parsed Preview**
5. **Send Once** — one publish
6. **Start Spam** — repeat at **Spam Frequency (s)** (min `0.05`)
7. **Stop Spam** on that row, or **Emergency Intercept: Cease All Spams** for every row

**＋ Append Frame Sequence** adds a row. Clone/delete with the row buttons. Fields lock while that row is spamming.

### Receive (right)

1. **Selective Filtering: Display IDs** — subscribe only to those frame IDs
2. Optional **Logical Payload Filters** — AND over payload bytes (`index`, `==` / `!=` / `<` / `>` / `<=` / `>=`, value `0–255`). Example: index `0`, `==`, `1` keeps frames whose first data byte is `1`
3. Log shows timestamp, command name, topic, data dec + hex
4. **Flush Buffer** clears the in-memory log (max 500 messages)
5. **Telemetry Refresh Rate** in the sidebar: `0` = manual only; otherwise the log auto-refreshes

With `master_loopback`, a sent frame only appears on the right after you select that same command ID.

## Quick checklist

1. Distrobox + workspace sourced
2. `master_loopback` (or `master_com`) running
3. `ros2 launch kalman_tools master_frame_panel.launch.py` running
4. Browser at `http://localhost:8501`
5. Sidebar **Online**
6. Send a catalog frame, select the same ID on the right, confirm a log row (loopback may drop ~50%)

## CLI smoke test (no UI)

```bash
ros2 topic pub --once /master_com/ros_to_master kalman_interfaces/msg/MasterMessage "{cmd: 0x20, data: [1]}"
ros2 topic echo /master_com/master_to_ros/x20 kalman_interfaces/msg/MasterMessage
```

## Troubleshooting

- **Offline / Resolving Link:** rosbridge not up, wrong port, or `start_rosbridge:=false` pointed at a dead server. Default panel port is `3001`, not `9090`.
- **Send works, receive empty:** pick the matching ID on the right. Loopback only republishes to `master_com/master_to_ros/{hex}`.
- **Console spam from loopback:** other nodes publishing `master_com/ros_to_master`. Kill extra drivers, or run loopback alone.
- **Frames missing with loopback:** default `loss_rate` `0.5`. Send again, or run with `ros2 run kalman_master master_loopback --ros-args -p loss_rate:=0.0`.
- **Port already in use:** change `streamlit_port` / `rosbridge_port` launch args.
