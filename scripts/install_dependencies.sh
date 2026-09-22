#!/usr/bin/env bash
# Install system dependencies for the SocialNav workspace on Ubuntu 22.04 + ROS 2 Humble
# (MASTER_PROMPT §42). Requires sudo.
set -euo pipefail

sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-core \
  ros-humble-nav2-controller \
  ros-humble-dwb-core \
  ros-humble-diagnostic-updater \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-xacro \
  ros-humble-robot-state-publisher

# rosdep for anything package.xml-declared that is not covered above.
if command -v rosdep >/dev/null 2>&1; then
  WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  rosdep install --from-paths "$WS/src" --ignore-src -r -y || true
fi

echo "[install_dependencies] done."
