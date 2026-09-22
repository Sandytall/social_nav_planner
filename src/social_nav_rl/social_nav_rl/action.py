"""Action mapping between the policy's normalized [-1, 1] space and robot velocity commands.

The policy outputs two numbers in [-1, 1]; this maps them to (v, w) within the robot's velocity
limits and clamps the per-step change to the acceleration limits. The policy therefore can
never command a velocity or acceleration outside the configured envelope - a hard limit the
safety supervisor further enforces downstream.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class ActionLimits:
    max_lin_vel: float = 0.8
    min_lin_vel: float = 0.0        # 0 = no reverse; set negative to allow backing up
    max_ang_vel: float = 1.2
    max_lin_accel: float = 1.0      # m/s^2
    max_ang_accel: float = 2.5      # rad/s^2
    dt: float = 0.05                # control period (s)


class ActionMapper:
    def __init__(self, limits: ActionLimits = None):
        self.limits = limits or ActionLimits()

    def denormalize(self, action, prev_v: float = None, prev_w: float = None):
        """Map a normalized action to (v, w). If prev_(v,w) given, clamp acceleration too.

        NaN/Inf in the action are treated as 0 (a stop request), never propagated."""
        lim = self.limits
        arr = np.asarray(action, dtype=np.float64)
        if not np.all(np.isfinite(arr)):
            v, w = lim.min_lin_vel, 0.0     # non-finite action -> stop request
        else:
            a0 = float(np.clip(arr[0], -1.0, 1.0))
            a1 = float(np.clip(arr[1], -1.0, 1.0))
            v = lim.min_lin_vel + 0.5 * (a0 + 1.0) * (lim.max_lin_vel - lim.min_lin_vel)
            w = a1 * lim.max_ang_vel
        if prev_v is not None and prev_w is not None:
            v = self._accel_clamp(prev_v, v, lim.max_lin_accel * lim.dt)
            w = self._accel_clamp(prev_w, w, lim.max_ang_accel * lim.dt)
        return float(v), float(w)

    def normalize(self, v: float, w: float):
        """Inverse map (v, w) -> normalized action, for imitation / warm-starting."""
        lim = self.limits
        span = max(lim.max_lin_vel - lim.min_lin_vel, 1e-6)
        a0 = 2.0 * (v - lim.min_lin_vel) / span - 1.0
        a1 = w / lim.max_ang_vel if lim.max_ang_vel else 0.0
        return float(np.clip(a0, -1.0, 1.0)), float(np.clip(a1, -1.0, 1.0))

    @staticmethod
    def _accel_clamp(prev: float, target: float, max_delta: float) -> float:
        return prev + float(np.clip(target - prev, -max_delta, max_delta))
