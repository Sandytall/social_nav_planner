"""Policy interface + baselines for the common evaluation harness.

A Policy is anything with ``act(obs) -> action`` in the env's normalized [-1,1]^2 space. This
lets the evaluator run RL and non-learned baselines through the exact same env/scenarios/seeds
(no privileged info for RL). Nav2 and the classical SocialNav baseline run in Gazebo through the
existing social_nav_benchmarks `--planner` path; the policies here run in the RL env.
"""
import math

import numpy as np

from social_nav_rl.observation import HUMAN_FEATURES, ROBOT_FEATURES


class StraightToGoalPolicy:
    """Non-learned baseline: turn toward the goal, slow when a human is near ahead. Reads only
    the public observation (goal bearing + nearest human), so it is a fair reference point."""

    def act(self, obs) -> np.ndarray:
        obs = np.asarray(obs, dtype=np.float32)
        goal_angle = float(obs[5])
        # Nearest human is the first block; its distance is block index 4.
        near_dist = float(obs[ROBOT_FEATURES + 4]) if obs.size > ROBOT_FEATURES + 4 else 1e3
        w = float(np.clip(goal_angle / (math.pi / 2.0), -1.0, 1.0))
        aligned = abs(goal_angle) < 0.5
        cautious = 0.0 < near_dist < 1.2
        v = 1.0 if aligned else 0.1
        if cautious:
            v = min(v, 0.2)
        return np.array([v, w], dtype=np.float32)


class ConstantPolicy:
    """Fixed action - useful as a control (e.g. a permanent-stop diagnostic)."""

    def __init__(self, v_norm=-1.0, w_norm=0.0):
        self._a = np.array([v_norm, w_norm], dtype=np.float32)

    def act(self, obs) -> np.ndarray:
        return self._a.copy()


def load_rl_policy(model_path: str):
    """Wrap a saved Stable-Baselines3 model as a Policy (lazy import; needs sb3 + the model)."""
    from stable_baselines3 import PPO
    model = PPO.load(model_path)

    class _RL:
        def act(self, obs):
            action, _ = model.predict(np.asarray(obs), deterministic=True)
            return action

    return _RL()
