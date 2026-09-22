"""Curriculum + domain randomization + seed splitting for RL training.

A curriculum is a list of levels (easy -> hard). Each level names the environments, scenario
types and difficulty it draws from; ``sample_episode`` randomly picks one per episode (domain
randomization) and assigns a seed from the requested split, so training never reuses
validation/test seeds. Levels advance when a rolling success rate clears a threshold. Pure /
config-driven; reuses the environment registry so it stays consistent with the sim.
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import List

from social_nav_rl.env import EpisodeConfig

# Disjoint seed bands so train / val / unseen-test never overlap.
SEED_SPLITS = {
    "train": (0, 100_000),
    "val": (100_000, 110_000),
    "test": (110_000, 130_000),
}

# Default 10-level curriculum (spec). Each entry: name, envs, scenarios, difficulty.
DEFAULT_LEVELS = [
    {"name": "L1_empty", "envs": ["urban"], "scenarios": ["empty"], "difficulty": "easy"},
    {"name": "L2_stationary", "envs": ["urban"], "scenarios": ["goal_blocked"], "difficulty": "easy"},
    {"name": "L3_single", "envs": ["urban"], "scenarios": ["crossing"], "difficulty": "easy"},
    {"name": "L4_multiple", "envs": ["urban", "warehouse"], "scenarios": ["normal"], "difficulty": "medium"},
    {"name": "L5_crossing", "envs": ["urban", "office"], "scenarios": ["crossing"], "difficulty": "medium"},
    {"name": "L6_approaching", "envs": ["urban", "hospital"], "scenarios": ["approaching"], "difficulty": "medium"},
    {"name": "L7_groups", "envs": ["plaza", "office"], "scenarios": ["group"], "difficulty": "medium"},
    {"name": "L8_dense", "envs": ["urban", "warehouse", "plaza"], "scenarios": ["dense_crowd"], "difficulty": "hard"},
    {"name": "L9_occlusion", "envs": ["factory", "warehouse", "hospital"], "scenarios": ["occlusion", "bottleneck"], "difficulty": "hard"},
    {"name": "L10_shift", "envs": ["urban", "factory", "warehouse", "hospital", "office", "plaza"],
     "scenarios": ["normal", "crossing", "group", "dense_crowd", "dynamic_obstacle"], "difficulty": "stress"},
]


@dataclass
class Curriculum:
    levels: List[dict] = field(default_factory=lambda: [dict(x) for x in DEFAULT_LEVELS])
    split: str = "train"
    advance_threshold: float = 0.8    # rolling success rate to move up
    window: int = 50                  # episodes averaged for advancement
    max_steps: int = 600
    level: int = 0
    _results: List[bool] = field(default_factory=list)

    def __post_init__(self):
        if self.split not in SEED_SPLITS:
            raise ValueError(f"unknown split {self.split!r}; use {list(SEED_SPLITS)}")

    def current(self) -> dict:
        return self.levels[min(self.level, len(self.levels) - 1)]

    def sample_episode(self, rng: random.Random) -> EpisodeConfig:
        lv = self.current()
        lo, hi = SEED_SPLITS[self.split]
        return EpisodeConfig(
            environment=rng.choice(lv["envs"]),
            scenario=rng.choice(lv["scenarios"]),
            difficulty=lv["difficulty"],
            max_steps=self.max_steps), rng.randrange(lo, hi)

    def record(self, success: bool) -> bool:
        """Record an episode result; return True if the level advanced."""
        self._results.append(bool(success))
        if len(self._results) < self.window:
            return False
        rate = sum(self._results[-self.window:]) / self.window
        if rate >= self.advance_threshold and self.level < len(self.levels) - 1:
            self.level += 1
            self._results.clear()
            return True
        return False


def load_curriculum(cfg: dict = None, split: str = "train") -> Curriculum:
    cfg = cfg or {}
    return Curriculum(
        levels=[dict(x) for x in cfg.get("levels", DEFAULT_LEVELS)],
        split=split,
        advance_threshold=cfg.get("advance_threshold", 0.8),
        window=cfg.get("window", 50),
        max_steps=cfg.get("max_steps", 600))
