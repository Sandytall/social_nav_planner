"""Fixed-size RL observation built from robot state + the existing human tracking/prediction.

Selection strategy (documented, and covered by tests):
  * Relevance = Euclidean distance from the robot to the human. Humans beyond `max_range` are
    dropped entirely (irrelevant), the rest are sorted nearest-first and the closest `n_humans`
    are kept. Ties break by human id for determinism.
  * The block for each kept human is filled; unused slots are zero-padded and their mask bit is
    0, so a variable human count (0, 5, 20, 50 ...) always yields the same vector length.
  * Per-feature-group ablation flags (velocity / prediction / uncertainty / ttc / social-zone /
    group) zero out that part of every human block, so ablation studies need no code change.

Inputs are plain dataclasses (RobotState, Human, Prediction), populated from social_nav_msgs
in the ROS layer, so this module stays ROS-free and unit-testable. No privileged/simulator-only
future information is used - predictions come from the existing classical predictor.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from social_nav_rl import features as F

ROBOT_FEATURES = 6          # v, w, goal_rel_x, goal_rel_y, goal_dist, goal_angle
HUMAN_FEATURES = 11         # see _human_block


@dataclass
class RobotState:
    x: float
    y: float
    yaw: float
    v: float                # current linear velocity
    w: float                # current angular velocity
    goal_x: float
    goal_y: float


@dataclass
class Human:
    id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0
    group_id: int = -1


@dataclass
class Prediction:
    """One human's predicted trajectory (from the classical predictor)."""
    human_id: int
    poses: List = field(default_factory=list)          # [(x, y), ...] future
    confidence: List = field(default_factory=list)     # per-step, in [0, 1]


@dataclass
class ObsConfig:
    n_humans: int = 5
    max_range: float = 8.0
    robot_radius: float = 0.45
    human_radius: float = 0.3
    use_velocity: bool = True
    use_prediction: bool = True
    use_uncertainty: bool = True
    use_ttc: bool = True
    use_social_zone: bool = True
    use_group: bool = True
    # Static-obstacle perception: a down-sampled lidar (n_lidar beams, normalised min-range per
    # beam) so the policy can avoid walls/racks. Computed by ray-cast in the mock and from the
    # real /scan in Gazebo (same beams). Appended AFTER the human blocks so human indexing and the
    # non-learned baselines are unaffected.
    use_lidar: bool = True
    n_lidar: int = 12
    lidar_range: float = 5.0


class ObservationAdapter:
    def __init__(self, cfg: ObsConfig = None):
        self.cfg = cfg or ObsConfig()

    @property
    def _n_lidar(self) -> int:
        return self.cfg.n_lidar if self.cfg.use_lidar else 0

    @property
    def size(self) -> int:
        return ROBOT_FEATURES + self.cfg.n_humans * HUMAN_FEATURES + self._n_lidar

    def build(self, robot: RobotState, humans: List[Human],
              predictions: Optional[Dict[int, Prediction]] = None, lidar=None):
        """Return (obs float32[size], mask float32[n_humans]) with mask=1 for real humans.

        `lidar` is an optional length-n_lidar array of raw beam ranges (m); it is normalised and
        appended after the human blocks. When omitted, the lidar slots read 1.0 (all clear).
        """
        predictions = predictions or {}
        obs = np.zeros(self.size, dtype=np.float32)
        mask = np.zeros(self.cfg.n_humans, dtype=np.float32)

        dist, bearing = F.goal_polar(robot.x, robot.y, robot.yaw, robot.goal_x, robot.goal_y)
        obs[0:ROBOT_FEATURES] = [robot.v, robot.w,
                                 *F.to_robot_frame(robot.goal_x, robot.goal_y,
                                                   robot.x, robot.y, robot.yaw),
                                 dist, bearing]

        chosen = self._select(robot, humans)
        for i, h in enumerate(chosen):
            lo = ROBOT_FEATURES + i * HUMAN_FEATURES
            obs[lo:lo + HUMAN_FEATURES] = self._human_block(robot, h, predictions.get(h.id))
            mask[i] = 1.0

        if self._n_lidar:
            base = ROBOT_FEATURES + self.cfg.n_humans * HUMAN_FEATURES
            if lidar is not None:
                from social_nav_rl.perception import normalize_lidar
                vals = normalize_lidar(lidar, self.cfg.lidar_range)
                obs[base:base + self.cfg.n_lidar] = vals[:self.cfg.n_lidar]
            else:
                obs[base:base + self.cfg.n_lidar] = 1.0   # unknown -> assume clear
        return F.safe_array(obs), mask

    def _select(self, robot: RobotState, humans: List[Human]) -> List[Human]:
        scored = []
        for h in humans:
            d = math.hypot(h.x - robot.x, h.y - robot.y)
            if d <= self.cfg.max_range:
                scored.append((d, h.id, h))
        scored.sort(key=lambda t: (t[0], t[1]))
        return [t[2] for t in scored[: self.cfg.n_humans]]

    def _human_block(self, robot: RobotState, h: Human, pred: Optional[Prediction]):
        c = self.cfg
        rx, ry = F.to_robot_frame(h.x, h.y, robot.x, robot.y, robot.yaw)
        dist = math.hypot(rx, ry)
        # Velocity in the robot frame (rotation only).
        cyaw, syaw = math.cos(-robot.yaw), math.sin(-robot.yaw)
        vfx = cyaw * h.vx - syaw * h.vy
        vfy = syaw * h.vx + cyaw * h.vy
        rel_head = F.relative_heading(robot.yaw, h.yaw)

        rel_pos = (h.x - robot.x, h.y - robot.y)
        rel_vel = (h.vx - robot.v * math.cos(robot.yaw), h.vy - robot.v * math.sin(robot.yaw))
        ttc = F.time_to_collision(rel_pos, rel_vel, c.robot_radius + c.human_radius)
        # Robot position in the human's frame -> anisotropic social-zone cost.
        hx_r, hy_r = F.to_robot_frame(robot.x, robot.y, h.x, h.y, h.yaw)
        zone = F.social_zone_cost(hx_r, hy_r)

        pred_close, pred_unc = self._prediction_features(robot, pred)

        block = [
            rx, ry,
            vfx if c.use_velocity else 0.0,
            vfy if c.use_velocity else 0.0,
            dist,
            rel_head,
            pred_close if c.use_prediction else 0.0,
            pred_unc if (c.use_prediction and c.use_uncertainty) else 0.0,
            (1.0 / (ttc + 1e-3)) if c.use_ttc else 0.0,   # inverse TTC: high => imminent
            zone if c.use_social_zone else 0.0,
            1.0 if (c.use_group and h.group_id >= 0) else 0.0,
        ]
        return block

    def _prediction_features(self, robot: RobotState, pred: Optional[Prediction]):
        """(closeness, uncertainty) from the classical prediction: how near the predicted path
        comes to the robot (1/(1+min_dist)), and the mean predicted uncertainty (1 - confidence)."""
        if pred is None or not pred.poses:
            return 0.0, 0.0
        min_d = min(math.hypot(px - robot.x, py - robot.y) for (px, py) in pred.poses)
        closeness = 1.0 / (1.0 + min_d)
        if pred.confidence:
            unc = 1.0 - float(np.mean(pred.confidence))
        else:
            unc = 0.0
        return closeness, max(0.0, min(1.0, unc))
