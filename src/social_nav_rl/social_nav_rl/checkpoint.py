"""Checkpoint metadata written next to a saved model (never uploaded anywhere).

Records what produced the model so an experiment is reproducible: algorithm, environment/split,
training steps, seed, the reward/observation/curriculum configs, and a git-independent project
version. Saved as <model>.meta.json beside the checkpoint.
"""
import json
import os
from dataclasses import asdict, dataclass, field

PROJECT_VERSION = "social_nav_rl-0.1.0"


@dataclass
class CheckpointMeta:
    algorithm: str = "ppo"
    environment: str = "urban"
    split: str = "train"
    training_steps: int = 0
    seed: int = 0
    project_version: str = PROJECT_VERSION
    reward_config: dict = field(default_factory=dict)
    obs_config: dict = field(default_factory=dict)
    curriculum: dict = field(default_factory=dict)
    extra: dict = field(default_factory=dict)

    def meta_path(self, model_path: str) -> str:
        return os.path.splitext(model_path)[0] + ".meta.json"

    def save(self, model_path: str) -> str:
        path = self.meta_path(model_path)
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)
        return path


def load_meta(model_path: str) -> dict:
    path = os.path.splitext(model_path)[0] + ".meta.json"
    with open(path) as f:
        return json.load(f)
