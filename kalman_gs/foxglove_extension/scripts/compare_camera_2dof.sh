#!/usr/bin/env bash
# Side-by-side compare: kalman_gs Camera 2DOF vs Foxglove Kalman Camera 2DOF.
# Run inside the ROS distrobox (same environment as gs.launch.py).

set -euo pipefail

EXT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST_HOME="${HOST_HOME:-/home/wiktor}"
ROSBRIDGE_URL="${ROSBRIDGE_URL:-ws://localhost:9065}"
GS_URL="${GS_URL:-http://localhost:3000}"

echo "=== Camera 2DOF comparison setup ==="
echo "Extension dir: $EXT_DIR"
echo "Host HOME:     $HOST_HOME"
echo "GS UI:         $GS_URL"
echo "Rosbridge:     $ROSBRIDGE_URL"
echo

# 1) Install extension into host Foxglove extensions dir (not distrobox HOME).
echo "--- Installing Foxglove extension into $HOST_HOME/.foxglove-studio ---"
cd "$EXT_DIR"
if [[ ! -d node_modules ]]; then
  npm install
fi
HOME="$HOST_HOME" npm run local-install
echo

# 2) Sanity: ports
echo "--- Port check ---"
if command -v ss >/dev/null 2>&1; then
  ss -ltn | grep -E ':9065|:3000' || echo "WARN: 3000/9065 not listening yet — start: ros2 launch kalman_gs gs.launch.py"
else
  echo "(ss not available; skip port check)"
fi
echo

# 3) Print manual checklist
cat <<EOF
--- Manual steps (you) ---

A. Ground station (old panel)
   1. Open $GS_URL
   2. Change a panel type to "Camera 2DOF"
   3. Do NOT Send until echo is running (step C)

B. Foxglove (new panel) — use foxglove_bridge :8765, NOT rosbridge :9065
   1. In another terminal (same ROS env): ./scripts/start_foxglove_bridge.sh
   2. From distrobox:  distrobox-host-exec foxglove-studio
      Or on host:      foxglove-studio
   3. Open connection → Foxglove WebSocket → ws://127.0.0.1:8765
   4. Add panel "Kalman Camera 2DOF"
   5. Status should say Ready / Publishing

C. Observer (this script starts typed echo below)
   NO fake publisher needed. Panels ARE the publishers.
   Echo may warn until first Send — that is normal.
   Then click Send (or Auto-send) in GS / Foxglove.

D. Checklist (alternate Send — do not both auto-send at once)
   [ ] Center + Send (GS)     → camera_id=0 yaw=90 pitch=90
   [ ] Center + Send (Fox)    → same
   [ ] Top-left corner + Send → yaw=0 pitch=0 (both UIs)
   [ ] Bottom-right + Send    → yaw=180 pitch=180 (both UIs)
   [ ] Auto-send 10 Hz ~5 s each → steady stream, similar rate

EOF

# #region agent log
DEBUG_LOG="${DEBUG_LOG:-/home/wiktor/programming/students_research_group/agh_space_systems/kalman_ws/.cursor/debug-c2c756.log}"
python3 -c "import json,time; open('$DEBUG_LOG','a').write(json.dumps({'sessionId':'c2c756','runId':'compare-script','hypothesisId':'H1','location':'compare_camera_2dof.sh','message':'starting_typed_echo','data':{'topic':'/camera_2dof/cmd','type':'kalman_interfaces/msg/Camera2Dof','fakeDataNeeded':False},'timestamp':int(time.time()*1000)})+'\n')" 2>/dev/null || true
# #endregion

# 4) Typed echo — waits without requiring an existing publisher
if command -v ros2 >/dev/null 2>&1; then
  MSG_TYPE="kalman_interfaces/msg/Camera2Dof"
  if ! ros2 interface show "$MSG_TYPE" >/dev/null 2>&1; then
    echo "ERROR: $MSG_TYPE not found in this environment."
    echo "Rebuild: colcon build --packages-select kalman_interfaces && source install/setup.bash"
    echo "Then re-run this script. Still NO fake data node — only rebuild + Send from UI."
    exit 1
  fi
  echo "--- Starting: ros2 topic echo /camera_2dof/cmd $MSG_TYPE ---"
  echo "Topic empty until you Send from GS or Foxglove. Waiting..."
  exec ros2 topic echo /camera_2dof/cmd "$MSG_TYPE"
else
  echo "ros2 not in PATH — start echo yourself after sourcing the workspace:"
  echo "  ros2 topic echo /camera_2dof/cmd kalman_interfaces/msg/Camera2Dof"
fi
