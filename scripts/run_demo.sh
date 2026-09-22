#!/usr/bin/env bash
# One-command SocialNav demo (MASTER_PROMPT §41): Gazebo + Nav2 + our controller, then a
# navigation goal is sent so you can watch the robot route to it (through the corridor
# gap, around the divider wall).
#
#   ./scripts/run_demo.sh              # Gazebo GUI, goal (2.5, 2.5)
#   GUI=false ./scripts/run_demo.sh    # headless
#   GOAL_X=-1 GOAL_Y=2 ./scripts/run_demo.sh
# Note: no `set -u` - sourcing ROS setup.bash references unbound vars (AMENT_TRACE_*).
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GUI="${GUI:-true}"
GOAL_X="${GOAL_X:-2.5}"
GOAL_Y="${GOAL_Y:-2.5}"

# --- environment (§56 troubleshooting) ---
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
  echo "[run_demo] shutting down (killing Gazebo + Nav2)..."
  # Gazebo Classic ignores polite signals; SIGKILL by cmdline match.
  pkill -9 -f gzclient 2>/dev/null || true
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f 'gazebo' 2>/dev/null || true
  pkill -9 -f "controller_server|planner_server|behavior_server|bt_navigator|lifecycle_manager|static_transform_publisher|spawn_entity|robot_state_publisher|human_publisher|human_markers|rviz2" 2>/dev/null || true
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

echo "[run_demo] launching Gazebo + Nav2 + SocialNavController (gui=$GUI)..."
ros2 launch social_nav_bringup demo.launch.py gui:="$GUI" &

# Optional: spawn simulated human(s) to watch social avoidance.
#   HUMANS="1.5,1.9,0,0" ./scripts/run_demo.sh      # one person standing on the route
#   HUMANS="0.0,0.4,0,0.3" ./scripts/run_demo.sh    # a person walking across
if [ -n "${HUMANS:-}" ]; then
  echo "[run_demo] publishing human(s): $HUMANS"
  ros2 run social_nav_tools human_publisher --ros-args \
    -p humans:="['${HUMANS}']" -p loop_period:="${HUMAN_LOOP:-0.0}" &
fi

# Social-zone markers for RViz (harmless if no people are present).
ros2 run social_nav_tools human_markers > /dev/null 2>&1 &

# RViz view (robot, costmap, plan, chosen path, social zones). RVIZ=false to skip.
# RViz is much lighter on the GPU than Gazebo's GUI - use GUI=false RVIZ=true to watch
# in RViz only if the Gazebo window is too heavy on the laptop.
if [ "${RVIZ:-true}" != "false" ]; then
  RVIZ_CFG="$WS/install/social_nav_rviz/share/social_nav_rviz/rviz/social_nav.rviz"
  rviz2 -d "$RVIZ_CFG" > /dev/null 2>&1 &
fi

# Wait until the WHOLE stack is active. bt_navigator activates last, so its lifecycle
# state going 'active' is the correct gate - sending before that gets the goal rejected.
echo "[run_demo] waiting for Nav2 to fully activate (bt_navigator)..."
for _ in $(seq 1 90); do
  if ros2 lifecycle get /bt_navigator 2>/dev/null | grep -qi "active"; then
    echo "[run_demo] bt_navigator active."
    break
  fi
  sleep 1
done
sleep 2

echo "[run_demo] sending goal ($GOAL_X, $GOAL_Y) in map frame..."
GOAL="{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y}}}}"
# Capture the output to a variable (NOT piped to grep -q, which closes the pipe early and
# gives ros2 a BrokenPipe -> false "not accepted" -> a spurious second goal). send_goal
# blocks until the goal finishes.
GOAL_OUT="$(ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$GOAL" 2>&1 || true)"
if echo "$GOAL_OUT" | grep -q "Goal finished with status: SUCCEEDED"; then
  echo "[run_demo] goal reached."
elif echo "$GOAL_OUT" | grep -q "Goal accepted"; then
  echo "[run_demo] goal accepted (see status above)."
else
  echo "[run_demo] goal was not accepted (Nav2 may still be starting)."
fi

echo "[run_demo] done. Ctrl-C to quit (or send more goals from another terminal)."
wait
