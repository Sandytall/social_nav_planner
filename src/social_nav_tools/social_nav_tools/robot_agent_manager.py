"""Secondary AMR agents: simple moving obstacles the main robot must avoid.

Per the multi-robot extension, extra robots are NOT part of the social problem - they are
plain dynamic obstacles. Each is a boxy, collision-bearing body the main robot's lidar sees,
teleported along a patrol route via /gazebo/set_entity_state (libgazebo_ros_state). Unlike the
pedestrians, they are NOT published on /social_nav/humans: they are handled by the ordinary
obstacle costmap layer, not the social layer.

  ros2 run social_nav_tools robot_agent_manager --ros-args -p num_agents:=2
"""
import math

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.parameter import Parameter

from social_nav_tools.pedestrian_model import pose_at_time
from social_nav_tools.robot_agent_model import plan_robot_agents

try:
    from gazebo_msgs.srv import SetEntityState, SpawnEntity
    _GAZEBO_SRV = True
except ImportError:  # gazebo_msgs missing: nothing to move (no bodies), stay quiet.
    _GAZEBO_SRV = False


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _amr_sdf(name: str) -> str:
    """A boxy AMR: kinematic (teleported, no physics) but WITH collision, so the lidar sees it
    and the planner treats it as an obstacle to route around."""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="body">
      <kinematic>true</kinematic>
      <gravity>false</gravity>
      <inertial><mass>10.0</mass>
        <inertia><ixx>0.5</ixx><iyy>0.5</iyy><izz>0.5</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="c"><pose>0 0 0.25 0 0 0</pose>
        <geometry><box><size>0.6 0.45 0.5</size></box></geometry></collision>
      <visual name="v"><pose>0 0 0.25 0 0 0</pose>
        <geometry><box><size>0.6 0.45 0.5</size></box></geometry>
        <material><ambient>0.85 0.55 0.10 1</ambient><diffuse>0.85 0.55 0.10 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


class RobotAgentManager(Node):
    def __init__(self):
        super().__init__("robot_agent_manager", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.declare_parameter("num_agents", 1)
        self.declare_parameter("speed", 0.4)
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("x_range", [3.0, 11.0])
        self.declare_parameter("lane_ys", [0.0, -1.0, 1.0])

        gp = self.get_parameter
        n = gp("num_agents").get_parameter_value().integer_value
        speed = gp("speed").get_parameter_value().double_value
        x_range = tuple(gp("x_range").get_parameter_value().double_array_value)
        lane_ys = list(gp("lane_ys").get_parameter_value().double_array_value)
        self.agents = plan_robot_agents(n, speed, x_range, lane_ys)
        self.get_logger().info(f"Planned {len(self.agents)} secondary AMR agent(s)")

        self._t0 = None
        self._spawned = set()
        self._inflight = {}
        self._set_client = None
        if _GAZEBO_SRV:
            self._set_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        else:
            self.get_logger().warn("gazebo_msgs not available; no AMR bodies will be spawned.")

        rate = gp("rate_hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

    def spawn_bodies(self):
        if not _GAZEBO_SRV:
            return
        client = self.create_client(SpawnEntity, "/spawn_entity")
        if not client.wait_for_service(timeout_sec=15.0):
            self.get_logger().warn("/spawn_entity unavailable; skipping AMR bodies.")
            return
        for a in self.agents:
            x, y, yaw, _, _ = pose_at_time(a, 0.0)
            req = SpawnEntity.Request()
            req.name = a.name
            req.xml = _amr_sdf(a.name)
            req.initial_pose.position.x = x
            req.initial_pose.position.y = y
            req.initial_pose.orientation = _yaw_to_quat(yaw)
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if future.result() is not None and future.result().success:
                self._spawned.add(a.name)
            else:
                self.get_logger().warn(f"spawn of {a.name} did not confirm success")
        self.get_logger().info(f"Spawned {len(self._spawned)} AMR body(ies) in Gazebo")

    def _tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:  # sim clock not up yet
            return
        if self._t0 is None:
            self._t0 = now
        if self._set_client is None or not self._set_client.service_is_ready():
            return
        t = (now - self._t0).nanoseconds * 1e-9
        for a in self.agents:
            if a.name not in self._spawned:
                continue
            if self._inflight.get(a.name) is not None and not self._inflight[a.name].done():
                continue
            x, y, yaw, _, _ = pose_at_time(a, t)
            req = SetEntityState.Request()
            req.state.name = a.name
            req.state.pose.position.x = x
            req.state.pose.position.y = y
            req.state.pose.orientation = _yaw_to_quat(yaw)
            req.state.reference_frame = "world"
            self._inflight[a.name] = self._set_client.call_async(req)


def main():
    rclpy.init()
    node = RobotAgentManager()
    try:
        node.spawn_bodies()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
