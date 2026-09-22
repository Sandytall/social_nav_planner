#!/usr/bin/env bash
# Generic one-command SocialNav environment demo, driven by the environment/scenario registry.
#
#   ENV=warehouse SCENARIO=crossing DIFFICULTY=hard ./scripts/run_env_demo.sh
#   ENV=urban SCENARIO=group NUM_ROBOTS=3 ./scripts/run_env_demo.sh
#   GUI=false RVIZ=true ENV=warehouse ./scripts/run_env_demo.sh     # lighter on the GPU
# Envs: ENV (urban|factory|warehouse), SCENARIO, DIFFICULTY (easy|medium|hard|stress), SEED,
#       NUM_ROBOTS, GUI, RVIZ. The goal is taken from the registry for the chosen environment.
# Note: no `set -u` - sourcing ROS setup.bash references unbound vars (AMENT_TRACE_*).
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV="${ENV:-urban}"; SCENARIO="${SCENARIO:-normal}"; DIFFICULTY="${DIFFICULTY:-medium}"
SEED="${SEED:-42}"; NUM_ROBOTS="${NUM_ROBOTS:-}"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source /usr/share/gazebo/setup.sh 2>/dev/null || true
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models"
export GAZEBO_MODEL_DATABASE_URI=""
if [ "${NV_OFFLOAD:-auto}" != "0" ] && command -v nvidia-smi >/dev/null 2>&1 \
    && nvidia-smi >/dev/null 2>&1; then
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
fi

# The goal for this environment/scenario comes from the same registry the launch uses.
GOAL_XY="$(python3 -c "from social_nav_tools.environments import generate_scenario as g; \
c=g('$ENV','$SCENARIO','$DIFFICULTY',$SEED); print(c['goal'][0], c['goal'][1])")"
read -r GOAL_X GOAL_Y <<< "$GOAL_XY"

cleanup() {
  echo ""; echo "[run_env_demo] shutting down..."
  pkill -9 -f gzclient 2>/dev/null || true
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f "controller_server|planner_server|behavior_server|bt_navigator|lifecycle_manager|static_transform_publisher|spawn_entity|robot_state_publisher|pedestrian_manager|robot_agent_manager|human_markers|rviz2" 2>/dev/null || true
  pkill -9 -f 'ros2 launch social_nav' 2>/dev/null || true
}
trap cleanup EXIT INT TERM
pkill -9 -f gzserver 2>/dev/null || true; pkill -9 -f gzclient 2>/dev/null || true
for _ in $(seq 1 10); do pgrep -x gzserver >/dev/null 2>&1 || break; sleep 0.5; done
sleep 1

NARGS=""; [ -n "$NUM_ROBOTS" ] && NARGS="num_robots:=$NUM_ROBOTS"
[ -n "${SENSOR:-}" ] && NARGS="$NARGS sensor:=$SENSOR"
echo "[run_env_demo] env=$ENV scenario=$SCENARIO difficulty=$DIFFICULTY seed=$SEED goal=($GOAL_X,$GOAL_Y)"
ros2 launch social_nav_bringup env_demo.launch.py \
  environment:="$ENV" scenario:="$SCENARIO" difficulty:="$DIFFICULTY" seed:="$SEED" \
  gui:="${GUI:-true}" rviz:="${RVIZ:-true}" $NARGS &

echo "[run_env_demo] waiting for Nav2 (bt_navigator)..."
for _ in $(seq 1 90); do
  if ros2 lifecycle get /bt_navigator 2>/dev/null | grep -qi "active"; then break; fi
  sleep 1
done
sleep 2

echo "[run_env_demo] sending goal ($GOAL_X, $GOAL_Y)..."
GOAL="{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y}}}}"
OUT="$(ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$GOAL" 2>&1 || true)"
echo "$OUT" | grep -q "SUCCEEDED" && echo "[run_env_demo] goal reached." \
  || echo "[run_env_demo] goal sent (see status above)."
echo "[run_env_demo] done. Ctrl-C to quit."
wait
