"""Pedestrian manager: one node that owns the pedestrian trajectories and drives BOTH ends of
the simulation from them.

  (a) Gazebo:  spawns a simple visual person per pedestrian and teleports it each tick via
               /gazebo/set_entity_state (libgazebo_ros_state) so people visibly walk.
  (b) planner: publishes the matching HumanState on /social_nav/humans every tick.

Because a single planned route feeds both, the person you see in Gazebo and the person the
planner reasons about are the same trajectory and cannot drift. The people are visual-only
(no collision), so the robot's lidar passes through them and the planner reacts to them purely
via /social_nav/humans and the social costmap layer, which is the point of a social planner.

Runs on sim time; stamps use the sim clock so the controller does not drop the humans as stale.

Example:
  ros2 run social_nav_tools pedestrian_manager --ros-args \
    -p num_humans:=3 -p seed:=42 -p behavior_profile:=mixed
"""
import math

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.parameter import Parameter
from social_nav_msgs.msg import HumanArray, HumanState

from social_nav_tools.pedestrian_model import Box, SpawnConfig, plan_pedestrians, pose_at_time

try:
    from gazebo_msgs.srv import SetEntityState, SpawnEntity
    _GAZEBO_SRV = True
except ImportError:  # gazebo_msgs missing: still publish humans, just skip the Gazebo bodies.
    _GAZEBO_SRV = False

# Default keep-outs matching urban.world (building facades + street furniture), map frame,
# as flat groups of four: xmin, ymin, xmax, ymax. Overridable with the `obstacles` param.
_DEFAULT_OBSTACLES = [
    -1.0, 1.6, 3.5, 4.6,     # building A
    7.5, 1.6, 13.0, 4.6,     # building B
    2.85, -1.35, 3.15, -1.05,  # lamp pole
    6.85, -1.35, 7.15, -1.05,  # bin
    9.4, 0.9, 10.6, 1.3,     # bench
]

# A small palette so the crowd is not monochrome; cycled by pedestrian index.
_COLORS = [(0.20, 0.40, 0.80), (0.80, 0.30, 0.20), (0.25, 0.65, 0.35),
           (0.75, 0.65, 0.20), (0.55, 0.30, 0.65)]


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _person_sdf(name: str, color) -> str:
    """A visual-only person (torso cylinder + head sphere). Kinematic + no collision so it is
    teleported cleanly and never seen by the lidar."""
    r, g, b = color
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="body">
      <kinematic>true</kinematic>
      <gravity>false</gravity>
      <self_collide>false</self_collide>
      <inertial><mass>1.0</mass>
        <inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="torso">
        <pose>0 0 0.6 0 0 0</pose>
        <geometry><cylinder><radius>0.22</radius><length>1.2</length></cylinder></geometry>
        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
      </visual>
      <visual name="head">
        <pose>0 0 1.36 0 0 0</pose>
        <geometry><sphere><radius>0.16</radius></sphere></geometry>
        <material><ambient>0.90 0.80 0.70 1</ambient><diffuse>0.90 0.80 0.70 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def _boxes_from_flat(flat):
    if len(flat) % 4 != 0:
        raise ValueError("obstacles must be a flat list of [xmin, ymin, xmax, ymax] groups")
    return tuple(Box(*flat[i:i + 4]) for i in range(0, len(flat), 4))


class PedestrianManager(Node):
    def __init__(self):
        super().__init__("pedestrian_manager", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.declare_parameter("num_humans", 3)
        self.declare_parameter("seed", 42)
        self.declare_parameter("behavior_profile", "mixed")
        self.declare_parameter("spawn_region", [1.0, 12.0, -0.5, 1.4])
        self.declare_parameter("min_separation", 1.0)
        self.declare_parameter("speed", 0.9)
        self.declare_parameter("group_size", 3)
        self.declare_parameter("group_spacing", 0.7)
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("robot_start", [0.0, 0.0])
        self.declare_parameter("robot_keepout", 1.5)
        self.declare_parameter("crossing_x", 5.5)
        self.declare_parameter("crossing_north", 2.2)
        self.declare_parameter("crossing_south", -3.0)
        self.declare_parameter("obstacles", _DEFAULT_OBSTACLES)
        # Gazebo world coords of the map origin (the robot's spawn pose). The urban launch
        # spawns the robot at the world origin, so the default is identity: world == map.
        self.declare_parameter("map_origin_in_world", [0.0, 0.0])

        gp = self.get_parameter
        region = tuple(gp("spawn_region").get_parameter_value().double_array_value)
        self.frame = gp("frame_id").get_parameter_value().string_value
        self.map_origin = tuple(gp("map_origin_in_world").get_parameter_value().double_array_value)
        obstacles = _boxes_from_flat(list(gp("obstacles").get_parameter_value().double_array_value))

        self.cfg = SpawnConfig(
            num_humans=gp("num_humans").get_parameter_value().integer_value,
            seed=gp("seed").get_parameter_value().integer_value,
            spawn_region=region,
            behavior_profile=gp("behavior_profile").get_parameter_value().string_value,
            min_separation=gp("min_separation").get_parameter_value().double_value,
            obstacles=obstacles,
            robot_start=tuple(gp("robot_start").get_parameter_value().double_array_value),
            robot_keepout=gp("robot_keepout").get_parameter_value().double_value,
            speed=gp("speed").get_parameter_value().double_value,
            group_size=gp("group_size").get_parameter_value().integer_value,
            group_spacing=gp("group_spacing").get_parameter_value().double_value,
            crossing_x=gp("crossing_x").get_parameter_value().double_value,
            crossing_north=gp("crossing_north").get_parameter_value().double_value,
            crossing_south=gp("crossing_south").get_parameter_value().double_value,
        )
        self.people = plan_pedestrians(self.cfg)
        self.get_logger().info(
            f"Planned {len(self.people)} pedestrian(s) "
            f"[{', '.join(p.behavior for p in self.people)}] with seed {self.cfg.seed}")

        self.pub = self.create_publisher(HumanArray, "/social_nav/humans", 10)
        self._t0 = None  # set on the first tick with a valid sim clock
        self._inflight = {}  # entity name -> pending set_entity_state future
        self._spawned = set()  # names confirmed present in Gazebo; only these get moved

        self._set_client = None
        if _GAZEBO_SRV:
            self._set_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        else:
            self.get_logger().warn(
                "gazebo_msgs not available; publishing /social_nav/humans only "
                "(no Gazebo bodies).")

        rate = gp("rate_hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

    def spawn_bodies(self):
        """Spawn one visual person per pedestrian. No-op (with a warning) if the Gazebo factory
        service never appears, so the planner-facing publishing still runs."""
        if not _GAZEBO_SRV:
            return
        client = self.create_client(SpawnEntity, "/spawn_entity")
        if not client.wait_for_service(timeout_sec=15.0):
            self.get_logger().warn("/spawn_entity unavailable; skipping Gazebo person bodies.")
            return
        for i, ped in enumerate(self.people):
            wx, wy, _, _, _ = self._world_pose(ped, 0.0)
            req = SpawnEntity.Request()
            req.name = ped.name
            req.xml = _person_sdf(ped.name, _COLORS[i % len(_COLORS)])
            req.initial_pose.position.x = wx
            req.initial_pose.position.y = wy
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if future.result() is None or not future.result().success:
                self.get_logger().warn(f"spawn of {ped.name} did not confirm success")
            else:
                # Mark present only after Gazebo confirms; the move timer (which fires while we
                # spin here) skips any body that is not yet spawned, avoiding "does not exist".
                self._spawned.add(ped.name)
        self.get_logger().info(f"Spawned {len(self._spawned)} pedestrian body(ies) in Gazebo")

    def _world_pose(self, ped, t):
        x, y, yaw, vx, vy = pose_at_time(ped, t)
        return (x + self.map_origin[0], y + self.map_origin[1], yaw, vx, vy)

    def _move_body(self, ped, wx, wy, yaw):
        if ped.name not in self._spawned:
            return  # body not confirmed spawned yet
        if self._set_client is None or not self._set_client.service_is_ready():
            return
        if self._inflight.get(ped.name) is not None and not self._inflight[ped.name].done():
            return  # previous move still pending; skip to avoid piling up requests
        req = SetEntityState.Request()
        req.state.name = ped.name
        req.state.pose.position.x = wx
        req.state.pose.position.y = wy
        req.state.pose.orientation = _yaw_to_quat(yaw)
        req.state.reference_frame = "world"
        self._inflight[ped.name] = self._set_client.call_async(req)

    def _tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:  # sim clock not up yet
            return
        if self._t0 is None:
            self._t0 = now
        t = (now - self._t0).nanoseconds * 1e-9

        msg = HumanArray()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame
        for ped in self.people:
            mx, my, myaw, vx, vy = pose_at_time(ped, t)  # map frame for the planner
            h = HumanState()
            h.id = ped.id
            h.pose.position.x = mx
            h.pose.position.y = my
            h.pose.orientation = _yaw_to_quat(myaw)
            h.velocity.linear.x = vx
            h.velocity.linear.y = vy
            h.confidence = 1.0
            h.tracking_age = t
            h.stationary = math.hypot(vx, vy) < 1e-6
            h.group_id = ped.group_id
            h.group_member = ped.group_id >= 0
            msg.humans.append(h)
            self._move_body(ped, mx + self.map_origin[0], my + self.map_origin[1], myaw)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PedestrianManager()
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
