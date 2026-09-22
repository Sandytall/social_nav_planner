"""Worker manager: one node that owns the factory-worker trajectories and drives BOTH ends of
the simulation from them, exactly like pedestrian_manager does for urban pedestrians.

  (a) Gazebo:  spawns a simple visual worker (hi-vis vest + hard hat) per worker and teleports
               it each tick via /gazebo/set_entity_state (libgazebo_ros_state).
  (b) planner: publishes the matching HumanState on /social_nav/humans every tick.

A single planned trajectory feeds both, so the worker you see in Gazebo and the worker the
planner reasons about are the same and cannot drift. Workers are visual-only (no collision),
so the robot's lidar passes through them and the planner reacts purely via /social_nav/humans
and the social costmap layer. Runs on sim time so stamps are not dropped as stale.

Example:
  ros2 run social_nav_tools worker_manager --ros-args \
    -p num_workstation:=3 -p num_crossing:=1 -p seed:=42
"""
import math

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from social_nav_msgs.msg import HumanArray, HumanState

from social_nav_tools.pedestrian_model import Box
from social_nav_tools.worker_model import (
    CROSSING,
    PATROL,
    WorkerConfig,
    plan_workers,
    pose_at_time,
)

try:
    from gazebo_msgs.srv import SetEntityState, SpawnEntity
    _GAZEBO_SRV = True
except ImportError:  # gazebo_msgs missing: still publish humans, just skip the Gazebo bodies.
    _GAZEBO_SRV = False

# Vest colours by role so the scene reads at a glance: workstation orange, crossing yellow,
# patrol blue. RGB in [0, 1].
_ROLE_COLOR = {
    "workstation": (0.90, 0.45, 0.10),
    CROSSING: (0.90, 0.80, 0.10),
    PATROL: (0.20, 0.45, 0.80),
}
_HELMET_COLOR = (0.95, 0.85, 0.15)


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _worker_sdf(name: str, color) -> str:
    """A visual-only worker: hi-vis torso + head + hard hat. Kinematic, no collision, so it is
    teleported cleanly and is invisible to the lidar (the planner sees it via /social_nav/humans
    only)."""
    r, g, b = color
    hr, hg, hb = _HELMET_COLOR
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
        <geometry><cylinder><radius>0.24</radius><length>1.2</length></cylinder></geometry>
        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
      </visual>
      <visual name="head">
        <pose>0 0 1.36 0 0 0</pose>
        <geometry><sphere><radius>0.16</radius></sphere></geometry>
        <material><ambient>0.90 0.80 0.70 1</ambient><diffuse>0.90 0.80 0.70 1</diffuse></material>
      </visual>
      <visual name="helmet">
        <pose>0 0 1.46 0 0 0</pose>
        <geometry><cylinder><radius>0.18</radius><length>0.12</length></cylinder></geometry>
        <material><ambient>{hr} {hg} {hb} 1</ambient><diffuse>{hr} {hg} {hb} 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def _boxes_from_flat(flat):
    if len(flat) % 4 != 0:
        raise ValueError("obstacles must be a flat list of [xmin, ymin, xmax, ymax] groups")
    return tuple(Box(*flat[i:i + 4]) for i in range(0, len(flat), 4))


def _points_from_flat(flat):
    if len(flat) % 2 != 0:
        raise ValueError("workstations must be a flat list of [x, y] pairs")
    return tuple((flat[i], flat[i + 1]) for i in range(0, len(flat), 2))


def _crossings_from_flat(flat):
    if len(flat) % 4 != 0:
        raise ValueError("crossing_routes must be a flat list of [nx, ny, sx, sy] groups")
    return tuple(((flat[i], flat[i + 1]), (flat[i + 2], flat[i + 3]))
                 for i in range(0, len(flat), 4))


class WorkerManager(Node):
    def __init__(self):
        super().__init__("worker_manager", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.declare_parameter("num_workstation", 3)
        self.declare_parameter("num_crossing", 1)
        self.declare_parameter("num_patrol", 0)
        self.declare_parameter("seed", 42)
        self.declare_parameter("speed", 0.9)
        self.declare_parameter("accel", 0.6)
        self.declare_parameter("dwell_time", 6.0)
        self.declare_parameter("dwell_jitter", 2.0)
        self.declare_parameter("speed_jitter", 0.15)
        self.declare_parameter("crossing_dwell", 1.5)
        self.declare_parameter("stations_per_worker", 3)
        self.declare_parameter("min_separation", 1.2)
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("robot_start", [0.0, 0.0])
        self.declare_parameter("robot_keepout", 1.5)
        self.declare_parameter("walkable", [-3.8, 10.8, -6.3, 6.3])
        self.declare_parameter("workstations", [], _DOUBLE_ARRAY)
        self.declare_parameter("crossing_routes", [], _DOUBLE_ARRAY)
        self.declare_parameter("obstacles", [], _DOUBLE_ARRAY)
        # Gazebo world coords of the map origin (the robot's spawn pose). The factory launch
        # spawns the robot at the world origin, so the default is identity: world == map.
        self.declare_parameter("map_origin_in_world", [0.0, 0.0])

        gp = self.get_parameter

        def darr(name):
            return list(gp(name).get_parameter_value().double_array_value)

        self.frame = gp("frame_id").get_parameter_value().string_value
        self.map_origin = tuple(darr("map_origin_in_world"))
        walkable = tuple(darr("walkable"))
        stations = _points_from_flat(darr("workstations"))
        crossings = _crossings_from_flat(darr("crossing_routes"))
        obstacles = _boxes_from_flat(darr("obstacles"))

        self.cfg = WorkerConfig(
            num_workstation=gp("num_workstation").get_parameter_value().integer_value,
            num_crossing=gp("num_crossing").get_parameter_value().integer_value,
            num_patrol=gp("num_patrol").get_parameter_value().integer_value,
            seed=gp("seed").get_parameter_value().integer_value,
            workstations=stations,
            crossing_routes=crossings,
            obstacles=obstacles,
            robot_start=tuple(gp("robot_start").get_parameter_value().double_array_value),
            robot_keepout=gp("robot_keepout").get_parameter_value().double_value,
            walkable=walkable,
            min_separation=gp("min_separation").get_parameter_value().double_value,
            speed=gp("speed").get_parameter_value().double_value,
            accel=gp("accel").get_parameter_value().double_value,
            dwell_time=gp("dwell_time").get_parameter_value().double_value,
            dwell_jitter=gp("dwell_jitter").get_parameter_value().double_value,
            speed_jitter=gp("speed_jitter").get_parameter_value().double_value,
            stations_per_worker=gp("stations_per_worker").get_parameter_value().integer_value,
            crossing_dwell=gp("crossing_dwell").get_parameter_value().double_value,
        )
        self.workers = plan_workers(self.cfg)
        self.get_logger().info(
            f"Planned {len(self.workers)} worker(s) "
            f"[{', '.join(w.role for w in self.workers)}] with seed {self.cfg.seed}")

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
        """Spawn one visual worker per plan. No-op (with a warning) if the Gazebo factory
        service never appears, so the planner-facing publishing still runs."""
        if not _GAZEBO_SRV or not self.workers:
            return
        client = self.create_client(SpawnEntity, "/spawn_entity")
        if not client.wait_for_service(timeout_sec=15.0):
            self.get_logger().warn("/spawn_entity unavailable; skipping Gazebo worker bodies.")
            return
        for worker in self.workers:
            wx, wy, _, _, _ = self._world_pose(worker, 0.0)
            color = _ROLE_COLOR.get(worker.role, (0.6, 0.6, 0.6))
            req = SpawnEntity.Request()
            req.name = worker.name
            req.xml = _worker_sdf(worker.name, color)
            req.initial_pose.position.x = wx
            req.initial_pose.position.y = wy
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if future.result() is None or not future.result().success:
                self.get_logger().warn(f"spawn of {worker.name} did not confirm success")
            else:
                self._spawned.add(worker.name)
        self.get_logger().info(f"Spawned {len(self._spawned)} worker body(ies) in Gazebo")

    def _world_pose(self, worker, t):
        x, y, yaw, vx, vy = pose_at_time(worker, t)
        return (x + self.map_origin[0], y + self.map_origin[1], yaw, vx, vy)

    def _move_body(self, worker, wx, wy, yaw):
        if worker.name not in self._spawned:
            return  # body not confirmed spawned yet
        if self._set_client is None or not self._set_client.service_is_ready():
            return
        if self._inflight.get(worker.name) is not None and not self._inflight[worker.name].done():
            return  # previous move still pending; skip to avoid piling up requests
        req = SetEntityState.Request()
        req.state.name = worker.name
        req.state.pose.position.x = wx
        req.state.pose.position.y = wy
        req.state.pose.orientation = _yaw_to_quat(yaw)
        req.state.reference_frame = "world"
        self._inflight[worker.name] = self._set_client.call_async(req)

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
        for worker in self.workers:
            mx, my, myaw, vx, vy = pose_at_time(worker, t)  # map frame for the planner
            h = HumanState()
            h.id = worker.id
            h.pose.position.x = mx
            h.pose.position.y = my
            h.pose.orientation = _yaw_to_quat(myaw)
            h.velocity.linear.x = vx
            h.velocity.linear.y = vy
            h.confidence = 1.0
            h.tracking_age = t
            h.stationary = math.hypot(vx, vy) < 1e-6
            h.group_id = -1
            msg.humans.append(h)
            self._move_body(worker, mx + self.map_origin[0], my + self.map_origin[1], myaw)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = WorkerManager()
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
