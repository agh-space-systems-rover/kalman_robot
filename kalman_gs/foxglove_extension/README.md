# Kalman Camera 2DOF — Foxglove extension

Pilot migration of the `kalman_gs` **Camera 2DOF** panel into a Foxglove custom panel.

Publishes `kalman_interfaces/Camera2Dof` on `/camera_2dof/cmd` (same contract as the React GS panel).

The original panel in `node_project` is unchanged.

## Prerequisites

- [Foxglove](https://foxglove.dev/) desktop or web app
- Node.js 18+
- ROS 2 stack with `rosbridge_websocket` on port **9065** (from `kalman_gs`) and optionally `camera_2dof_driver`

## Install (local)

```bash
cd src/kalman_robot/kalman_gs/foxglove_extension
npm install
npm run local-install
```

This copies the built extension into `~/.foxglove-studio/extensions/`. Reload Foxglove (or restart the app) after install.

Rebuild after code changes:

```bash
npm run local-install
```

## Connect and use

1. Start rosbridge (and GS if desired):

   ```bash
   ros2 launch kalman_gs gs.launch.py
   ```

   Bring up the camera 2DOF driver as in your usual GS / master launch.

2. Start **foxglove_bridge** (separate from GS rosbridge `:9065`):

   ```bash
   # inside distrobox / same ROS env as gs.launch.py
   ./scripts/start_foxglove_bridge.sh
   # or: ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
   ```

3. In Foxglove: **Open connection** → **Foxglove WebSocket** → `ws://127.0.0.1:8765`

   | App | Connection type | URL |
   |-----|-----------------|-----|
   | React GS | (built-in roslib) | `ws://…:9065` rosbridge |
   | Foxglove desktop | **Foxglove WebSocket** | `ws://127.0.0.1:8765` foxglove_bridge |

   Do **not** point Foxglove at `:9065` / Rosbridge — Studio speaks `foxglove.sdk.v1`; rosbridge is a different protocol → generic “not reachable” even when TCP works.

   Direct live connections need a **Developer** seat (Free plan includes 3).

4. Add panel **Kalman Camera 2DOF** from the panel list.

5. Drag the pad, click **Send** (or enable **Auto-send**). Confirm:

   ```bash
   ros2 topic echo /camera_2dof/cmd
   ```

## Message mapping

Pad position `x,y ∈ [0,1]` maps to:

| Field       | Value                         |
|-------------|-------------------------------|
| `camera_id` | `0`                           |
| `yaw`       | `round(x * 180)` (degrees)    |
| `pitch`     | `round(y * 180)` (degrees)    |

UI yaw/pitch labels show values relative to center (`*180 - 90`), matching the GS panel.

## Compare vs kalman_gs panel

One-shot helper (installs extension into **host** `HOME`, then echoes the topic):

```bash
# Inside distrobox, with gs.launch.py already running (rosbridge :9065 + GS :3000)
./scripts/compare_camera_2dof.sh
```

Or install only for host Foxglove:

```bash
HOME=/home/wiktor npm run local-install
distrobox-host-exec foxglove-studio
```

| UI | Open | Panel name |
|----|------|------------|
| Old | http://localhost:3000 | Camera 2DOF |
| New | Foxglove desktop + **Foxglove WebSocket** `ws://127.0.0.1:8765` | Kalman Camera 2DOF |

Expected publish (both): `/camera_2dof/cmd` — center → `yaw: 90`, `pitch: 90`; corners → `0/0` or `180/180`.

## Notes

- Needs a **live** Rosbridge (or other publish-capable) connection. File / bag playback has no publish — status line will say so.
- Rosbridge JSON publish does not require a datatype map; **foxglove_bridge** does — the panel passes `kalman_interfaces/msg/Camera2Dof` field definitions on advertise.
- Distrobox: `npm run local-install` without `HOME=/home/wiktor` installs under the container home (e.g. `.distrobox/.foxglove-studio`). Host Foxglove will not see that — use the script or `HOME=/home/wiktor`.
