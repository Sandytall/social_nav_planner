"""Gymnasium environment for RL social navigation.

Wires the pure core (observation / action / safety / reward) around a pluggable simulation
backend:

  * MockBackend - a fast, deterministic kinematic sim that REUSES the project's
    social_nav_tools.pedestrian_model + environments, so humans move exactly as they do in
    Gazebo and scenarios come from the same registry. This is the accelerated training mode.
  * GazeboBackend - the interface to the real simulation (steps the actual robot/sensors/
    prediction); implemented in the ROS node, run by the user. Not importable without ROS.

The env exposes a fixed-size observation (core features + the human mask) and a continuous
[-1, 1]^2 action. Every command passes the SafetySupervisor before reaching the sim, so the
policy never bypasses the hard constraints. Deterministic given a seed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

from social_nav_rl.action import ActionLimits, ActionMapper
from social_nav_rl.observation import (
    Human, ObsConfig, ObservationAdapter, Prediction, RobotState)
from social_nav_rl.reward import RewardComputer, RewardConfig
from social_nav_rl.safety import SafetyConfig, SafetySupervisor
from social_nav_rl import features as F


@dataclass
class EpisodeConfig:
    environment: str = "urban"
    scenario: str = "normal"
    difficulty: str = "medium"
    max_steps: int = 600
    arrival_radius: float = 0.6
    robot_radius: float = 0.45
    human_radius: float = 0.3
    prediction_horizon: float = 3.0


class MockBackend:
    """Kinematic sim: diff-drive robot + registry-driven humans (via pedestrian_model)."""

    def __init__(self, ep: EpisodeConfig, dt: float, obs_cfg=None):
        self.ep = ep
        self.dt = dt
        self.obs_cfg = obs_cfg
        self._people = []
        self._t = 0.0
        self._obstacles = []          # pedestrian-model obstacles
        self._world_obstacles = []    # real static geometry from the Gazebo world (robot collision)
        self.robot = None
        self.goal = (0.0, 0.0)

    def reset(self, seed: int):
        # Import here so a missing social_nav_tools (bare unit env) fails only when used.
        from social_nav_tools.environments import generate_scenario
        from social_nav_tools.pedestrian_model import Box, SpawnConfig, plan_pedestrians

        cfg = generate_scenario(self.ep.environment, self.ep.scenario, self.ep.difficulty, seed)
        p = cfg["pedestrian"]
        sc = SpawnConfig(
            num_humans=p["num_humans"], seed=p["seed"],
            spawn_region=tuple(p["spawn_region"]), behavior_profile=p["behavior_profile"],
            obstacles=tuple(Box(*p["obstacles"][i:i + 4])
                            for i in range(0, len(p["obstacles"]), 4)),
            robot_start=tuple(p["robot_start"]), speed=p.get("speed", 0.9),
            crossing_x=p.get("crossing_x", 5.5),
            crossing_north=p.get("crossing_north", 2.2),
            crossing_south=p.get("crossing_south", -3.0))
        self._people = plan_pedestrians(sc)
        self._obstacles = [(b.xmin, b.ymin, b.xmax, b.ymax) for b in sc.obstacles]
        self._world_obstacles = [tuple(b) for b in cfg.get("world_obstacles", [])]
        self._t = 0.0
        sx, sy, syaw = cfg["robot_start"]
        self.robot = {"x": sx, "y": sy, "yaw": syaw, "v": 0.0, "w": 0.0}
        self.goal = tuple(cfg["goal"])
        return self._humans_now()

    def step(self, v: float, w: float):
        r = self.robot
        r["x"] += v * math.cos(r["yaw"]) * self.dt
        r["y"] += v * math.sin(r["yaw"]) * self.dt
        r["yaw"] = F.wrap_angle(r["yaw"] + w * self.dt)
        r["v"], r["w"] = v, w
        self._t += self.dt
        return self._humans_now()

    def _humans_now(self):
        from social_nav_tools.pedestrian_model import pose_at_time
        out = []
        for ped in self._people:
            x, y, yaw, vx, vy = pose_at_time(ped, self._t)
            out.append(Human(id=ped.id, x=x, y=y, vx=vx, vy=vy, yaw=yaw,
                             group_id=getattr(ped, "group_id", -1)))
        return out

    def predictions(self, humans: List[Human]) -> Dict[int, Prediction]:
        """Constant-velocity prediction as a stand-in for the classical predictor (the Gazebo
        backend subscribes to the real /social_nav prediction instead). Confidence decays."""
        preds = {}
        steps = max(1, int(self.ep.prediction_horizon / self.dt))
        for h in humans:
            poses, conf = [], []
            for k in range(1, steps + 1):
                poses.append((h.x + h.vx * self.dt * k, h.y + h.vy * self.dt * k))
                conf.append(max(0.0, 1.0 - k / steps))
            preds[h.id] = Prediction(human_id=h.id, poses=poses, confidence=conf)
        return preds

    def obstacle_hit(self, x: float, y: float, radius: float) -> bool:
        # Robot collisions test the REAL world geometry (racks/walls), matching Gazebo's lidar.
        for (xmin, ymin, xmax, ymax) in self._world_obstacles:
            if (xmin - radius) <= x <= (xmax + radius) and (ymin - radius) <= y <= (ymax + radius):
                return True
        return False

    def lidar(self):
        """Down-sampled ray-cast lidar (raw ranges) against the world obstacles, or None if off."""
        if not self.obs_cfg or not getattr(self.obs_cfg, "use_lidar", False):
            return None
        from social_nav_rl.perception import raycast_lidar
        r = self.robot
        return raycast_lidar(r["x"], r["y"], r["yaw"], self._world_obstacles,
                             n_beams=self.obs_cfg.n_lidar, max_range=self.obs_cfg.lidar_range)


try:
    import gymnasium as gym
    from gymnasium import spaces
    _GYM = True
except ImportError:  # pragma: no cover - env only used where gymnasium is installed
    _GYM = False
    gym = object  # type: ignore


class SocialNavEnv(gym.Env if _GYM else object):
    """Gymnasium env. observation = [core features (obs adapter) | human mask]."""

    metadata = {"render_modes": []}

    def __init__(self, ep: EpisodeConfig = None, obs_cfg: ObsConfig = None,
                 limits: ActionLimits = None, reward_cfg: RewardConfig = None,
                 safety_cfg: SafetyConfig = None, backend=None, episode_sampler=None,
                 safe_filter: bool = True):
        if not _GYM:
            raise ImportError("gymnasium is required for SocialNavEnv "
                              "(pip install gymnasium)")
        super().__init__()
        self.ep = ep or EpisodeConfig()
        # Optional () -> (EpisodeConfig, seed) for domain randomization / curriculum. When set
        # with the default MockBackend, each reset re-samples the scenario (harder levels as the
        # curriculum advances).
        self.episode_sampler = episode_sampler
        self.adapter = ObservationAdapter(obs_cfg or ObsConfig())
        self.limits = limits or ActionLimits()
        self.mapper = ActionMapper(self.limits)
        self.reward = RewardComputer(reward_cfg or RewardConfig())
        self.safety = SafetySupervisor(safety_cfg or SafetyConfig())
        self.safe_filter = safe_filter
        self.backend = backend or MockBackend(self.ep, self.limits.dt, obs_cfg=self.adapter.cfg)
        self._steps = 0
        self._prev_v = self._prev_w = 0.0
        self._prev_goal_dist = 0.0
        self._prev_success = None       # last episode's success, fed to the sampler on reset

        obs_dim = self.adapter.size + self.adapter.cfg.n_humans
        self.observation_space = spaces.Box(-1e3, 1e3, shape=(obs_dim,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        seed = 0 if seed is None else int(seed)
        if self.episode_sampler is not None and isinstance(self.backend, MockBackend):
            self.ep, seed = self.episode_sampler(self._prev_success)
            self.backend = MockBackend(self.ep, self.limits.dt)
        humans = self.backend.reset(seed)
        self._steps = 0
        self._prev_v = self._prev_w = 0.0
        self._prev_goal_dist = self._goal_dist()
        return self._obs(humans), {}

    def step(self, action):
        self.safety.log.clear()
        v_cmd, w_cmd = self.mapper.denormalize(action, self._prev_v, self._prev_w)
        humans = self.backend._humans_now()
        preds = self.backend.predictions(humans)
        if self.safe_filter:
            signals_pre = self._risk_signals(humans)
            decision = self.safety.filter(v_cmd, w_cmd, {
                "prev_v": self._prev_v, "prev_w": self._prev_w, "t": self._steps * self.limits.dt,
                **signals_pre})
            v, w, safety_kind = decision.v, decision.w, decision.kind
        else:
            # No supervisor: the policy's command is executed as-is, so PPO learns from the action
            # it actually took (the supervisor otherwise rewrites v/w and breaks credit assignment,
            # driving the "crawl" under dense humans). Safety is taught via the reward's collision
            # penalties; the supervisor stays a deploy-time backstop (safe_filter=True at eval).
            v, w, safety_kind = v_cmd, w_cmd, "off"

        prev_x, prev_y = self.backend.robot["x"], self.backend.robot["y"]
        humans = self.backend.step(v, w)
        self._steps += 1

        signals, events = self._transition(humans, v, w, prev_x, prev_y)
        reward, components = self.reward.compute(signals)
        terminated = bool(events["reached"] or events["collision"] or events["human_collision"])
        truncated = self._steps >= self.ep.max_steps
        self._prev_v, self._prev_w = v, w
        self._prev_goal_dist = signals["goal_dist"]
        if terminated or truncated:
            self._prev_success = bool(events["reached"] and not events["collision"]
                                      and not events["human_collision"])

        info = {"reward_components": components, "events": events,
                "safety": safety_kind, "interventions": len(self.safety.log)}
        return self._obs(humans, preds), reward, terminated, truncated, info

    # -- helpers ---------------------------------------------------------------
    def _goal_dist(self):
        r, g = self.backend.robot, self.backend.goal
        return math.hypot(g[0] - r["x"], g[1] - r["y"])

    def _robot_state(self):
        r, g = self.backend.robot, self.backend.goal
        return RobotState(r["x"], r["y"], r["yaw"], r["v"], r["w"], g[0], g[1])

    def _obs(self, humans, preds=None):
        if preds is None:
            preds = self.backend.predictions(humans)
        lidar = self.backend.lidar() if hasattr(self.backend, "lidar") else None
        obs, mask = self.adapter.build(self._robot_state(), humans, preds, lidar=lidar)
        return np.concatenate([obs, mask]).astype(np.float32)

    def _clearances(self, humans):
        r = self.backend.robot
        return [math.hypot(h.x - r["x"], h.y - r["y"]) for h in humans]

    def _risk_signals(self, humans):
        r = self.backend.robot
        rv = (r["v"] * math.cos(r["yaw"]), r["v"] * math.sin(r["yaw"]))
        min_ttc = F.BIG_TTC
        for h in humans:
            ttc = F.time_to_collision((h.x - r["x"], h.y - r["y"]),
                                      (h.vx - rv[0], h.vy - rv[1]),
                                      self.ep.robot_radius + self.ep.human_radius)
            min_ttc = min(min_ttc, ttc)
        clr = self._clearances(humans)
        return {"min_ttc": min_ttc, "min_clearance": (min(clr) if clr else None),
                "obstacle_imminent": self.backend.obstacle_hit(
                    r["x"], r["y"], self.ep.robot_radius + 0.05)}

    def _transition(self, humans, v, w, prev_x, prev_y):
        r = self.backend.robot
        clr = self._clearances(humans)
        min_clr = min(clr) if clr else None
        human_collision = bool(min_clr is not None and
                               min_clr < self.ep.robot_radius + self.ep.human_radius)
        collision = self.backend.obstacle_hit(r["x"], r["y"], self.ep.robot_radius)
        goal_dist = self._goal_dist()
        reached = goal_dist < self.ep.arrival_radius
        risk = self._risk_signals(humans)
        zone = 0.0
        group = 0.0
        for h in humans:
            hx_r, hy_r = F.to_robot_frame(r["x"], r["y"], h.x, h.y, h.yaw)
            z = F.social_zone_cost(hx_r, hy_r)
            zone += z
            if h.group_id >= 0:
                group += z
        step_len = math.hypot(r["x"] - prev_x, r["y"] - prev_y)
        osc = (w * self._prev_w < 0.0 and abs(w) > 0.1 and abs(self._prev_w) > 0.1)
        signals = {
            "prev_goal_dist": self._prev_goal_dist, "goal_dist": goal_dist, "reached": reached,
            "collision": collision, "human_collision": human_collision,
            "min_ttc": risk["min_ttc"], "min_clearance": min_clr,
            "social_zone_sum": zone, "group_intrusion": group,
            "step_len": step_len, "straight_step": max(0.0, self._prev_goal_dist - goal_dist),
            "v": v, "w": w, "prev_v": self._prev_v, "prev_w": self._prev_w, "osc_switch": osc,
        }
        events = {"reached": reached, "collision": collision,
                  "human_collision": human_collision, "min_clearance": min_clr,
                  "min_ttc": risk["min_ttc"]}
        return signals, events
