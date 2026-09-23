"""Policy interface + baselines for the common evaluation harness.

A Policy is anything with ``act(obs) -> action`` in the env's normalized [-1,1]^2 space (and an
optional ``reset()`` called at each episode start). This lets the evaluator run RL and non-learned
baselines through the exact same env/scenarios/seeds (no privileged info for RL). Nav2 and the
classical SocialNav planner in Gazebo run through social_nav_benchmarks `--planner`; the policies
here run in the accelerated RL env, so RL vs. the classical Social-Force baseline is a matched,
same-env, same-seed comparison.
"""
import math

import numpy as np

from social_nav_rl.observation import HUMAN_FEATURES, ROBOT_FEATURES


class FrameStacker:
    """Replicates SB3 ``VecFrameStack`` ordering for single-env evaluation: the buffer is
    zero-filled on reset with the newest frame in the last slot, and each push rolls left and
    writes the newest frame last. Using this at eval time keeps a frame-stacked policy's input
    identical to what it saw during (vectorized) training - a mismatch would silently feed the
    net garbage."""

    def __init__(self, n_stack: int, obs_dim: int):
        self.n = int(n_stack)
        self.d = int(obs_dim)
        self.buf = np.zeros(self.n * self.d, dtype=np.float32)

    def reset(self, obs) -> np.ndarray:
        self.buf[:] = 0.0
        self.buf[-self.d:] = np.asarray(obs, dtype=np.float32)
        return self.buf.copy()

    def push(self, obs) -> np.ndarray:
        self.buf = np.roll(self.buf, -self.d)
        self.buf[-self.d:] = np.asarray(obs, dtype=np.float32)
        return self.buf.copy()


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


class SocialForcePolicy:
    """Classical Social-Force local planner. An attractive pull toward the goal plus an
    exponential repulsion from every visible human, summed in the robot frame and mapped to
    (v, w). Reads only the public observation (+ the appended human mask), so it competes on the
    same footing as the RL policy - a genuine reactive planner, unlike the naive straight line.

    Tunables mirror the classic Helbing/Molnar social-force parameters (repulsion strength +
    range); defaults are set for the 0.8 m/s, ~0.45 m robot in these scenarios.
    """

    def __init__(self, n_humans=5, goal_gain=1.0, human_gain=2.2, human_sigma=0.8, turn_gain=1.3,
                 max_range=8.0):
        self.n_humans = n_humans
        self.goal_gain = goal_gain
        self.human_gain = human_gain
        self.human_sigma = human_sigma
        self.turn_gain = turn_gain
        self.max_range = max_range

    def act(self, obs) -> np.ndarray:
        obs = np.asarray(obs, dtype=np.float32)
        # Human blocks sit right after the robot block; the mask is always the last n_humans entries
        # (a lidar block, if any, is between them and is ignored here). Passed in, not inferred,
        # because an appended lidar makes size-based inference of n_humans ambiguous.
        n_humans = self.n_humans
        mask = obs[obs.size - n_humans:] if n_humans else np.zeros(0, dtype=np.float32)
        # Attractive: unit vector toward the goal in the robot frame (x forward, y left).
        gx, gy = float(obs[2]), float(obs[3])
        gd = math.hypot(gx, gy) + 1e-6
        fx, fy = self.goal_gain * gx / gd, self.goal_gain * gy / gd
        # Repulsive: exponential push away from each real human.
        for i in range(n_humans):
            if mask[i] < 0.5:
                continue
            lo = ROBOT_FEATURES + i * HUMAN_FEATURES
            hx, hy = float(obs[lo]), float(obs[lo + 1])
            d = math.hypot(hx, hy)
            if d < 1e-3 or d > self.max_range:
                continue
            mag = self.human_gain * math.exp(-d / self.human_sigma)
            fx -= mag * hx / d
            fy -= mag * hy / d
        # Map the resultant force (robot frame) to (v, w) in [-1, 1].
        heading = math.atan2(fy, fx)                       # desired turn vs. current heading
        w = float(np.clip(self.turn_gain * heading / (math.pi / 2.0), -1.0, 1.0))
        fmag = math.hypot(fx, fy) + 1e-6
        v = float(np.clip(fx / fmag, 0.0, 1.0))            # full speed only when net force is ahead
        return np.array([v, w], dtype=np.float32)


class ConstantPolicy:
    """Fixed action - useful as a control (e.g. a permanent-stop diagnostic)."""

    def __init__(self, v_norm=-1.0, w_norm=0.0):
        self._a = np.array([v_norm, w_norm], dtype=np.float32)

    def act(self, obs) -> np.ndarray:
        return self._a.copy()


def load_rl_policy(model_path: str):
    """Wrap a saved Stable-Baselines3 model as a Policy (lazy import; needs sb3 + the model).

    If the model was trained with frame stacking (its observation is a multiple of the env's),
    the wrapper transparently stacks frames at eval time via FrameStacker, resetting the stack at
    each episode start (through ``reset()``), so no caller needs to know the stack depth.
    """
    from stable_baselines3 import PPO, SAC
    # Pick the algorithm from the sibling .meta.json (PPO.load can't read a SAC/LSTM zip). Fall back
    # to trying PPO then SAC so older checkpoints without meta still load.
    import json
    import os
    algo_map = {"ppo": PPO, "sac": SAC}
    try:
        from sb3_contrib import RecurrentPPO
        algo_map["recurrent_ppo"] = RecurrentPPO
    except ImportError:
        RecurrentPPO = None
    meta = os.path.splitext(model_path)[0] + ".meta.json"
    algo = None
    if os.path.exists(meta):
        try:
            with open(meta) as f:
                algo = json.load(f).get("algorithm")
        except Exception:
            algo = None
    if algo in algo_map:
        model = algo_map[algo].load(model_path)
    else:
        try:
            model = PPO.load(model_path)
        except Exception:
            model = SAC.load(model_path)
    recurrent = model.__class__.__name__ == "RecurrentPPO"
    model_dim = int(np.prod(model.observation_space.shape))

    class _RL:
        def __init__(self):
            self._stacker = None
            self._need_reset = True
            self._lstm = None            # RecurrentPPO hidden state, carried across the episode

        def reset(self):
            self._need_reset = True
            self._lstm = None

        def act(self, obs):
            obs = np.asarray(obs, dtype=np.float32)
            if model_dim != obs.size and obs.size > 0 and model_dim % obs.size == 0:
                if self._stacker is None:
                    self._stacker = FrameStacker(model_dim // obs.size, obs.size)
                x = self._stacker.reset(obs) if self._need_reset else self._stacker.push(obs)
            else:
                x = obs
            if recurrent:
                ep_start = np.ones((1,), dtype=bool) if self._need_reset else np.zeros((1,), dtype=bool)
                action, self._lstm = model.predict(
                    x, state=self._lstm, episode_start=ep_start, deterministic=True)
            else:
                action, _ = model.predict(x, deterministic=True)
            self._need_reset = False
            return action

    return _RL()
