#!/usr/bin/env bash
# Force-stop everything the SocialNav demo starts.
# Gazebo Classic's gzserver/gzclient often ignore Ctrl-C and linger, holding the master
# port (11345) so the next launch fails with "Unable to start server". Run this to recover.
#
#   ./scripts/stop.sh
echo "[stop] killing Gazebo + Nav2 + SocialNav processes..."

# Gazebo (GUI + server) - SIGKILL, they ignore polite signals.
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f 'gazebo'  2>/dev/null

# ros2 launch + our nodes.
pkill -9 -f 'ros2 launch social_nav' 2>/dev/null
pkill -9 -f 'controller_server|planner_server|behavior_server|bt_navigator|lifecycle_manager' 2>/dev/null
pkill -9 -f 'robot_state_publisher|spawn_entity|static_transform_publisher|human_publisher|human_markers' 2>/dev/null
pkill -9 -f 'rviz2' 2>/dev/null

sleep 2
LEFT="$(pgrep -f 'gzserver|gzclient' | grep -v $$ || true)"
if [ -z "$LEFT" ]; then
  echo "[stop] clean - no Gazebo processes remain."
else
  echo "[stop] WARNING: still running: $LEFT (try: sudo kill -9 $LEFT)"
fi
