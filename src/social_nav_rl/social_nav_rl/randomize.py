"""Domain randomization for the accelerated mock env, to shrink the mock->Gazebo gap.

The ranges are meant to bracket the differences a system-ID pass (scripts/sysid.py) finds between
the mock and Gazebo: lidar noise, control latency, velocity-tracking error, and robot/human speed.
A policy trained across this distribution is far more likely to transfer than one trained on a
single nominal dynamics (the mock is 1.0 on crossing, Gazebo ~0.2).

Everything is off by default (all-zero ranges == the deterministic mock). `--domain-rand` in the
trainer turns on a sensible default profile; tune the ranges from sysid measurements.
"""
from dataclasses import dataclass


@dataclass
class DRConfig:
    enabled: bool = False
    lidar_noise: float = 0.0        # Gaussian stddev (m) added to each mock lidar beam (per step)
    lidar_dropout: float = 0.0      # per-beam probability of a missed return (reads max_range)
    action_latency: int = 0         # execute the action from up to this many steps ago (sampled/ep)
    vel_tracking_err: float = 0.0   # per-step Gaussian noise on executed (v,w): imperfect tracking
    speed_scale: float = 0.0        # robot speed systematic offset 1 +/- this (per episode)
    human_speed_scale: float = 0.0  # pedestrian speed 1 +/- this (per episode)


def default_profile() -> DRConfig:
    """A GENTLE, physically-realistic starting profile for --domain-rand: a well-tuned diff-drive
    base tracks velocity within a few percent with ~1 control step of lag, and a good lidar has
    cm-level noise. Kept mild on purpose - aggressive DR taxes nominal success heavily and needs a
    much larger timestep budget. Widen it (esp. latency / tracking) only from real sysid.py
    measurements, ideally as a fine-tune on top of a clean (no-DR) policy rather than from scratch."""
    return DRConfig(enabled=True, lidar_noise=0.02, lidar_dropout=0.01, action_latency=1,
                    vel_tracking_err=0.05, speed_scale=0.05, human_speed_scale=0.1)


class Randomizer:
    """Per-episode dynamics + per-step sensor/actuation perturbations, used by SocialNavEnv when DR
    is on. `rng` is a random.Random so the whole thing is reproducible given a seed."""

    def __init__(self, cfg: DRConfig, rng):
        self.cfg = cfg
        self.rng = rng
        self.reset_episode()

    def reset_episode(self):
        c, r = self.cfg, self.rng
        # systematic per-episode offsets (robot-to-robot + crowd-to-crowd variation)
        self.latency = r.randint(0, c.action_latency) if c.action_latency else 0
        self.speed = 1.0 + r.uniform(-c.speed_scale, c.speed_scale)
        self.human_speed = 1.0 + r.uniform(-c.human_speed_scale, c.human_speed_scale)
        self._buf = []

    def delay_action(self, v, w):
        """Action actually executed: apply sampled control latency (a queue delay), the per-episode
        speed offset, and per-step velocity-tracking noise. Models a real drive base's imperfect,
        laggy velocity tracking that the deterministic mock lacks."""
        self._buf.append((v, w))
        i = max(0, len(self._buf) - 1 - self.latency)
        ev, ew = self._buf[i]
        scale = self.speed
        if self.cfg.vel_tracking_err > 0:
            scale *= 1.0 + self.rng.gauss(0.0, self.cfg.vel_tracking_err)
        return ev * scale, ew * scale

    def noisy_lidar(self, ranges, max_range):
        """Add per-beam range noise + random missed returns to the mock's clean ray-cast lidar."""
        if ranges is None or (self.cfg.lidar_noise <= 0 and self.cfg.lidar_dropout <= 0):
            return ranges
        out = list(ranges)
        for i in range(len(out)):
            if self.cfg.lidar_dropout and self.rng.random() < self.cfg.lidar_dropout:
                out[i] = max_range
            elif self.cfg.lidar_noise:
                out[i] = max(0.0, out[i] + self.rng.gauss(0.0, self.cfg.lidar_noise))
        return out
