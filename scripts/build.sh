#!/usr/bin/env bash
# Build the SocialNav workspace (MASTER_PROMPT §42).
# No `set -u`: sourcing ROS setup.bash references unbound vars (AMENT_TRACE_*).
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --symlink-install "$@"
echo "[build] done. Source it with:  source $WS/install/setup.bash"
