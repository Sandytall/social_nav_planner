"""GazeboBackend: high-fidelity RL backend that steps the REAL simulation.

Implements the same interface as MockBackend, so SocialNavEnv(backend=GazeboBackend(...)) trains
/ evaluates against the actual robot, sensors and human pipeline. It publishes /cmd_vel, reads
/odom + /social_nav/humans + the classical prediction, and teleports the robot to the scenario
start each episode via /gazebo/set_entity_state.

Requires a running RL sim stack WITHOUT the Nav2 controller (the RL policy owns /cmd_vel):
    ros2 launch social_nav_bringup rl_sim.launch.py environment:=factory scenario:=crossing
Because Gazebo Classic runs about real-time, this backend steps at ~1x - use it for
high-fidelity fine-tuning / evaluation; bulk training uses the accelerated MockBackend.

Needs rclpy + the project messages, so it is imported only when actually used (never in the
bare unit tests); the message->observation conversion lives in ros_convert.py and IS unit-tested.
"""
import math

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

from social_nav_rl import ros_convert as RC
from social_nav_rl.observation import Human, Prediction

try:
    from social_nav_msgs.msg import HumanArray, HumanPredictionArray
except ImportError:  # pragma: no cover
    HumanArray = HumanPredictionArray = None
try:
    from gazebo_msgs.srv import SetEntityState
except ImportError:  # pragma: no cover
    SetEntityState = None


def _yaw_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class GazeboBackend:
    def __init__(self, ep, dt, obs_cfg=None, entity="social_bot", node=None):
        if not rclpy.ok():
            rclpy.init()
        self.ep = ep
        self.dt = dt
        self.obs_cfg = obs_cfg
        self.entity = entity
        self.node = node or Node("social_nav_rl_backend")
        n = self.node
        # use_sim_time is pre-declared on every rclpy node, so set it rather than re-declare
        # (re-declaring raises ParameterAlreadyDeclaredException).
        if not n.has_parameter("use_sim_time"):
            n.declare_parameter("use_sim_time", True)
        else:
            n.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self._odom = None
        self._humans_msg = None
        self._pred_msg = None
        self._scan_min = math.inf
        self._scan = None
        self.robot = {"x": 0.0, "y": 0.0, "yaw": 0.0, "v": 0.0, "w": 0.0}
        self.goal = (0.0, 0.0)

        self._cmd = n.create_publisher(Twist, "cmd_vel", 10)
        n.create_subscription(Odometry, "odom", self._on_odom, 20)
        if HumanArray is not None:
            n.create_subscription(HumanArray, "/social_nav/humans", self._on_humans, 10)
        if HumanPredictionArray is not None:
            n.create_subscription(HumanPredictionArray, "/social_nav/prediction",
                                  self._on_pred, 10)
        n.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self._set_client = (n.create_client(SetEntityState, "/gazebo/set_entity_state")
                            if SetEntityState is not None else None)

    # -- callbacks -------------------------------------------------------------
    def _on_odom(self, m):
        self._odom = m

    def _on_humans(self, m):
        self._humans_msg = m

    def _on_pred(self, m):
        self._pred_msg = m

    def _on_scan(self, m):
        self._scan = m
        vals = [r for r in m.ranges if math.isfinite(r) and r > 0.0]
        self._scan_min = min(vals) if vals else math.inf

    def lidar(self):
        """Down-sample the real /scan into the same beams the mock ray-cast uses, or None if off."""
        if (not self.obs_cfg or not getattr(self.obs_cfg, "use_lidar", False)
                or self._scan is None):
            return None
        from social_nav_rl.perception import downsample_scan
        m = self._scan
        return downsample_scan(list(m.ranges), m.angle_min, m.angle_increment,
                               n_beams=self.obs_cfg.n_lidar, max_range=self.obs_cfg.lidar_range)

    # -- backend interface -----------------------------------------------------
    def reset(self, seed):
        from social_nav_tools.environments import generate_scenario
        cfg = generate_scenario(self.ep.environment, self.ep.scenario, self.ep.difficulty, seed)
        self.goal = tuple(cfg["goal"])
        sx, sy, syaw = cfg["robot_start"]
        self._teleport(sx, sy, syaw)
        self._spin(0.5)                       # let fresh odom/humans arrive
        self._sync_robot()
        self._report_inputs()
        return self._humans_now()

    def _report_inputs(self):
        """One line per episode confirming the model's live inputs are actually arriving."""
        have_odom = self._odom is not None
        have_scan = self._scan is not None
        nh = len(RC.humans_from_array(self._humans_msg)) if self._humans_msg is not None else 0
        lid = self.lidar()
        if lid is not None and len(lid):
            lid_str = f"OK (nearest {min(lid):.2f}m, {len(lid)} beams)"
        elif not self.obs_cfg or not getattr(self.obs_cfg, "use_lidar", False):
            lid_str = "DISABLED in obs_cfg"
        else:
            lid_str = "MISSING (no /scan -> model is obstacle-BLIND!)"
        self.node.get_logger().info(
            f"[inputs] odom={'OK' if have_odom else 'MISSING'} "
            f"scan={'OK' if have_scan else 'MISSING'} "
            f"humans_perceived={nh} lidar={lid_str}")

    def step(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._cmd.publish(msg)
        self._spin(self.dt)                   # advance ~dt of sim time at ~1x
        self._sync_robot()
        return self._humans_now()

    def _humans_now(self):
        if self._humans_msg is None:
            return []
        return RC.humans_from_array(self._humans_msg)

    def predictions(self, humans):
        if self._pred_msg is not None:
            return RC.predictions_from_array(self._pred_msg)
        # No prediction node running -> constant-velocity fallback (documented stand-in).
        steps = max(1, int(self.ep.prediction_horizon / self.dt))
        out = {}
        for h in humans:
            poses = [(h.x + h.vx * self.dt * k, h.y + h.vy * self.dt * k)
                     for k in range(1, steps + 1)]
            out[h.id] = Prediction(h.id, poses, [max(0.0, 1.0 - k / steps)
                                                 for k in range(1, steps + 1)])
        return out

    def obstacle_hit(self, x, y, radius):
        return self._scan_min < radius

    # -- helpers ---------------------------------------------------------------
    def _sync_robot(self):
        if self._odom is not None:
            self.robot = RC.odom_to_robot(self._odom, self.goal)

    def _teleport(self, x, y, yaw):
        if self._set_client is None or not self._set_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().warn("/gazebo/set_entity_state unavailable; robot not reset")
            return
        req = SetEntityState.Request()
        req.state.name = self.entity
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.orientation = _yaw_quat(yaw)
        req.state.reference_frame = "world"
        fut = self._set_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)

    def _spin(self, seconds):
        clock = self.node.get_clock()
        t0 = clock.now()
        while (clock.now() - t0).nanoseconds * 1e-9 < seconds:
            rclpy.spin_once(self.node, timeout_sec=0.02)
