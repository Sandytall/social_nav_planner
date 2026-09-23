"""Safety supervisor: the hard-constraint layer between the RL policy and cmd_vel.

The RL policy never drives the robot directly. Every candidate (v, w) passes through here and
is rejected/modified/braked/stopped when it would be unsafe, mirroring the classical planner's
safety states. Each intervention is recorded (kind + reason) so failures are explainable and
the intervention rate is measurable. Pure and unit-testable; the ROS node feeds it live signals.
"""
from dataclasses import dataclass, field
from typing import List

import numpy as np

STOP, CAUTIOUS, SLOW, NAN, NONE = "emergency_stop", "cautious", "slow", "nan_action", "none"


@dataclass
class SafetyConfig:
    max_lin_vel: float = 0.8
    max_ang_vel: float = 1.2
    max_lin_accel: float = 1.0
    max_ang_accel: float = 2.5
    dt: float = 0.05
    ttc_stop: float = 1.0        # s: below -> emergency stop
    ttc_slow: float = 3.0        # s: below -> scale speed down
    clearance_stop: float = 0.4  # m: below -> emergency stop
    clearance_slow: float = 1.0  # m: below -> scale speed down
    human_timeout: float = 1.0   # s: older human data -> cautious cap
    cautious_speed: float = 0.25  # m/s cap when data is stale / prediction invalid


@dataclass
class SafetyDecision:
    v: float
    w: float
    intervened: bool
    kind: str
    reason: str


class SafetySupervisor:
    def __init__(self, cfg: SafetyConfig = None):
        self.cfg = cfg or SafetyConfig()
        self.log: List[dict] = []

    def filter(self, v: float, w: float, state: dict = None) -> SafetyDecision:
        """Return a SafetyDecision with a safe (v, w). `state` carries live signals:
        prev_v, prev_w, min_ttc, min_clearance, human_data_age, prediction_valid,
        obstacle_imminent."""
        s = state or {}
        c = self.cfg

        # 1. Invalid action -> full stop.
        if not (np.isfinite(v) and np.isfinite(w)):
            return self._decide(0.0, 0.0, NAN, "non-finite action", s)

        # 2. Imminent collision -> emergency stop.
        min_ttc = float(s.get("min_ttc", 1e3))
        min_clr = s.get("min_clearance")
        if s.get("obstacle_imminent") or min_ttc < c.ttc_stop or \
                (min_clr is not None and min_clr < c.clearance_stop):
            return self._decide(0.0, 0.0, STOP,
                                f"ttc={min_ttc:.2f} clr={min_clr}", s)

        # 3. Stale human data or invalid prediction -> cautious cap.
        stale = float(s.get("human_data_age", 0.0)) > c.human_timeout
        if stale or not s.get("prediction_valid", True):
            v = min(v, c.cautious_speed)
            v, w = self._limit(v, w, s)
            return self._decide(v, w, CAUTIOUS,
                                "stale humans" if stale else "invalid prediction", s)

        # 4. Close/high-TTC -> proportional slow-down.
        scale = 1.0
        if min_ttc < c.ttc_slow:
            scale = min(scale, min_ttc / c.ttc_slow)
        if min_clr is not None and min_clr < c.clearance_slow:
            scale = min(scale, max(0.0, (min_clr - c.clearance_stop) /
                                   (c.clearance_slow - c.clearance_stop)))
        if scale < 0.999:
            v *= scale
            v, w = self._limit(v, w, s)
            return self._decide(v, w, SLOW, f"scale={scale:.2f}", s)

        # 5. Routine limit/accel clamp (not counted as a safety intervention).
        v, w = self._limit(v, w, s)
        return self._decide(v, w, NONE, "", s, intervened=False)

    def _limit(self, v, w, s):
        c = self.cfg
        v = float(np.clip(v, -c.max_lin_vel, c.max_lin_vel))
        w = float(np.clip(w, -c.max_ang_vel, c.max_ang_vel))
        if "prev_v" in s:
            v = s["prev_v"] + float(np.clip(v - s["prev_v"],
                                            -c.max_lin_accel * c.dt, c.max_lin_accel * c.dt))
        if "prev_w" in s:
            w = s["prev_w"] + float(np.clip(w - s["prev_w"],
                                            -c.max_ang_accel * c.dt, c.max_ang_accel * c.dt))
        return v, w

    def _decide(self, v, w, kind, reason, s, intervened=True):
        if intervened:
            self.log.append({"kind": kind, "reason": reason,
                             "t": s.get("t"), "v": v, "w": w})
        return SafetyDecision(v, w, intervened, kind, reason)

    def intervention_counts(self) -> dict:
        counts = {}
        for e in self.log:
            counts[e["kind"]] = counts.get(e["kind"], 0) + 1
        return counts
