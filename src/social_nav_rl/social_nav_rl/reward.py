"""Modular, per-component reward for social navigation.

Each component is computed and weighted separately and returned in a dict, so every term can be
configured, logged and ablated on its own - there is no single opaque formula. The env fills a
`signals` dict each step (what happened physically); this turns it into weighted components and
a total. Nothing here reads the simulator directly, so it is fully unit-testable.
"""
from dataclasses import dataclass, field
from typing import Dict

DEFAULT_WEIGHTS = {
    "goal_progress": 1.0,        # per metre closer to the goal
    "goal_completion": 50.0,     # one-off on reaching the goal
    "collision": -50.0,          # obstacle collision (per event)
    "human_collision": -60.0,    # collision with a person (worse)
    "ttc": -2.0,                 # dangerous predicted encounter
    "clearance": 0.5,            # maintaining comfortable human distance
    "social_zone": -1.0,         # intruding on personal/social space
    "path_efficiency": -0.2,     # excessive detour vs the straight step
    "time": -0.02,               # per step, discourages dawdling
    "smoothness": -0.1,          # linear-accel magnitude
    "angular_smoothness": -0.1,  # angular-accel magnitude
    "stopping": -0.1,            # stationary while not at the goal
    "oscillation": -0.2,         # left/right direction switching
    "group": -0.5,               # group-space intrusion
}

DEFAULT_PARAMS = {
    "ttc_danger": 3.0,       # s: below this, TTC is penalised
    "comfort_dist": 1.2,     # m: personal space to maintain
    "stop_speed": 0.05,      # m/s: below this counts as stopped
    "wait_clearance": 1.5,   # m: if a human is this close, stopping is WAITING (not dawdling) and
    #                          is not penalised - lets the robot yield to a blocking group/crosser
    #                          in a narrow corridor with no room to go around.
}


@dataclass
class RewardConfig:
    weights: Dict[str, float] = field(default_factory=lambda: dict(DEFAULT_WEIGHTS))
    params: Dict[str, float] = field(default_factory=lambda: dict(DEFAULT_PARAMS))


class RewardComputer:
    def __init__(self, cfg: RewardConfig = None):
        self.cfg = cfg or RewardConfig()

    def _raw(self, s: dict) -> Dict[str, float]:
        p = self.cfg.params
        g = lambda k, d=0.0: float(s.get(k, d))       # noqa: E731 - tiny local getter
        min_ttc = g("min_ttc", 1e3)
        clr = s.get("min_clearance")
        reached = bool(s.get("reached"))
        speed = abs(g("v"))
        # Stopping is only "dawdling" (penalised) in open space; stopping with a human within
        # wait_clearance is legitimate YIELDING (e.g. letting a group/crosser pass in a tight aisle).
        waiting = clr is not None and clr <= p["wait_clearance"]
        return {
            "goal_progress": g("prev_goal_dist") - g("goal_dist"),
            "goal_completion": 1.0 if reached else 0.0,
            "collision": 1.0 if s.get("collision") else 0.0,
            "human_collision": 1.0 if s.get("human_collision") else 0.0,
            "ttc": max(0.0, 1.0 - min_ttc / p["ttc_danger"]) if min_ttc < p["ttc_danger"] else 0.0,
            "clearance": min(0.0, clr - p["comfort_dist"]) if clr is not None else 0.0,
            "social_zone": g("social_zone_sum"),
            "path_efficiency": max(0.0, g("step_len") - g("straight_step")),
            "time": 1.0,
            "smoothness": abs(g("v") - g("prev_v")),
            "angular_smoothness": abs(g("w") - g("prev_w")),
            "stopping": 1.0 if (speed < p["stop_speed"] and not reached and not waiting) else 0.0,
            "oscillation": 1.0 if s.get("osc_switch") else 0.0,
            "group": g("group_intrusion"),
        }

    def compute(self, signals: dict):
        """Return (total_reward, components) where components[k] is the WEIGHTED contribution."""
        raw = self._raw(signals)
        components = {k: self.cfg.weights.get(k, 0.0) * v for k, v in raw.items()}
        return float(sum(components.values())), components
