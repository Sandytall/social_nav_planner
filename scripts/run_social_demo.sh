#!/usr/bin/env bash
# One-command SocialNav urban demo: Gazebo (urban.world) + Nav2 + our controller + walking
# pedestrians, then a navigation goal is sent so you can watch the robot route down the
# sidewalk past the buildings while a pedestrian crosses its path.
#
#   ./scripts/run_social_demo.sh                    # Gazebo GUI, crossing scenario, goal (12, 0)
#   GUI=false ./scripts/run_social_demo.sh          # headless
#   SCENARIO=walking ./scripts/run_social_demo.sh   # pedestrians walk along the sidewalk
#   GOAL_X=10 GOAL_Y=0 ./scripts/run_social_demo.sh
# Note: no `set -u` - sourcing ROS setup.bash references unbound vars (AMENT_TRACE_*).
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GUI="${GUI:-true}"
SCENARIO="${SCENARIO:-crossing}"
# Goal must sit within the global costmap's rolling window (~10 m half-width, centred on the
# robot's start), so it stays under 10 m from the origin spawn. 9 m still passes the crossing
# and the sidewalk pinch, exercising the social behaviour.
GOAL_X="${GOAL_X:-9.0}"
GOAL_Y="${GOAL_Y:-0.0}"

# --- environment ---
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source /usr/share/gazebo/setup.sh 2>/dev/null || true
# Only the built-in models (sun/ground_plane) are needed; set a CLEAN path so gzclient does
# not scan inherited paths (e.g. turtlebot3) that print "missing model.config".
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models"
# Disable the (dead) online model database, or gzclient hangs at "Preparing your world"
# waiting on a network fetch. This is the usual cause of that hang.
export GAZEBO_MODEL_DATABASE_URI=""

_CLEANED=0
cleanup() {
  [ "$_CLEANED" = 1 ] && return
  _CLEANED=1
  echo ""
  echo "[run_social_demo] shutting down (killing Gazebo + Nav2)..."
  # Gazebo Classic ignores polite signals; SIGKILL by cmdline match.
  pkill -9 -f gzclient 2>/dev/null || true
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f 'gazebo' 2>/dev/null || true
  pkill -9 -f "controller_server|planner_server|behavior_server|bt_navigator|lifecycle_manager|static_transform_publisher|spawn_entity|robot_state_publisher|pedestrian_manager|human_publisher|human_markers|rviz2" 2>/dev/null || true
  pkill -9 -f 'ros2 launch social_nav' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Pre-clean any zombie Gazebo from a previous run and WAIT for the master port to free,
# else the new gzserver dies with "Unable to start server [bind: Address already in use]".
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f gzserver 2>/dev/null || true
for _ in $(seq 1 10); do
  pgrep -x gzserver >/dev/null 2>&1 || break
  sleep 0.5
done
sleep 1

echo "[run_social_demo] launching urban demo (gui=$GUI, scenario=$SCENARIO)..."
ros2 launch social_nav_bringup urban_demo.launch.py gui:="$GUI" scenario:="$SCENARIO" &

# Wait until the WHOLE stack is active. bt_navigator activates last, so its lifecycle
# state going 'active' is the correct gate - sending before that gets the goal rejected.
echo "[run_social_demo] waiting for Nav2 to fully activate (bt_navigator)..."
for _ in $(seq 1 90); do
  if ros2 lifecycle get /bt_navigator 2>/dev/null | grep -qi "active"; then
    echo "[run_social_demo] bt_navigator active."
    break
  fi
  sleep 1
done
sleep 2

echo "[run_social_demo] sending goal ($GOAL_X, $GOAL_Y) in map frame..."
GOAL="{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y}}}}"
# Capture the output (NOT piped to grep -q, which closes the pipe early and gives ros2 a
# BrokenPipe -> false "not accepted" -> a spurious second goal). send_goal blocks until done.
GOAL_OUT="$(ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$GOAL" 2>&1 || true)"
if echo "$GOAL_OUT" | grep -q "Goal finished with status: SUCCEEDED"; then
  echo "[run_social_demo] goal reached."
elif echo "$GOAL_OUT" | grep -q "Goal accepted"; then
  echo "[run_social_demo] goal accepted (see status above)."
else
  echo "[run_social_demo] goal was not accepted (Nav2 may still be starting)."
fi

echo "[run_social_demo] done. Ctrl-C to quit (or send more goals from another terminal)."
wait
