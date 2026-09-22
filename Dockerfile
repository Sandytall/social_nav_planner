# SocialNav Planner — reproducible build.
# Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic + Nav2.
FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-nav2-costmap-2d \
      ros-humble-diagnostic-updater \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-plugins \
      ros-humble-xacro \
      ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src ./src
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Source ROS + the workspace on every shell.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /ws/install/setup.bash' >> /root/.bashrc && \
    echo 'source /usr/share/gazebo/setup.sh 2>/dev/null || true' >> /root/.bashrc && \
    echo 'export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH}' >> /root/.bashrc && \
    echo 'export GAZEBO_MODEL_DATABASE_URI=' >> /root/.bashrc

# Default: headless demo (override CMD for GUI once X11/GPU are shared, see docker-compose).
CMD ["bash", "-lc", "GUI=false ./scripts/run_demo.sh"]
