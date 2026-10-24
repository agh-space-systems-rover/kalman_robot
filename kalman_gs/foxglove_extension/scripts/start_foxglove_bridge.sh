#!/usr/bin/env bash
# Start foxglove_bridge on :8765 for Foxglove desktop (Foxglove WebSocket connection).
# Keep kalman_gs rosbridge on :9065 for the React GS — both can run together.
#
# Run inside the same ROS distrobox/container as gs.launch.py.

set -eo pipefail
# ROS setup.bash reads optional vars (e.g. AMENT_TRACE_SETUP_FILES); -u breaks sourcing.

# Temporarily allow unset vars while sourcing ament/ROS setup scripts.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
WS_ROOT="${_KALMAN_WS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)}"
if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$WS_ROOT/install/setup.bash"
fi
set -u

if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  echo "foxglove_bridge not installed. In the container:"
  echo "  sudo apt-get install -y ros-humble-foxglove-bridge"
  exit 1
fi

PORT="${FOXGLOVE_BRIDGE_PORT:-8765}"
echo "Starting foxglove_bridge on port $PORT"
echo "In Foxglove: Open connection → Foxglove WebSocket → ws://127.0.0.1:${PORT}"
echo "(Do NOT use Rosbridge / port 9065 for the Foxglove app — that path is flaky for Studio.)"
echo

exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:="$PORT"
