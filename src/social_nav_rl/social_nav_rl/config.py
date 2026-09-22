"""Load experiment configuration from YAML into the core dataclasses.

Experiments are configured in files, not hardcoded in Python: observation, action limits,
reward weights and safety thresholds each have a YAML block. Unknown keys are ignored and a
missing file falls back to code defaults, so a partial config is always valid.
"""
import os

import yaml

from social_nav_rl.action import ActionLimits
from social_nav_rl.observation import ObsConfig
from social_nav_rl.reward import RewardConfig
from social_nav_rl.safety import SafetyConfig


def _load(path):
    if path and os.path.isfile(path):
        with open(path) as f:
            return yaml.safe_load(f) or {}
    return {}


def _only_fields(cls, data):
    """Keep only keys that are real dataclass fields of `cls`."""
    fields = getattr(cls, "__dataclass_fields__", {})
    return {k: v for k, v in (data or {}).items() if k in fields}


def load_obs_config(path=None) -> ObsConfig:
    return ObsConfig(**_only_fields(ObsConfig, _load(path)))


def load_action_limits(path=None) -> ActionLimits:
    return ActionLimits(**_only_fields(ActionLimits, _load(path)))


def load_safety_config(path=None) -> SafetyConfig:
    return SafetyConfig(**_only_fields(SafetyConfig, _load(path)))


def load_reward_config(path=None) -> RewardConfig:
    """reward.yaml is {weights: {...}, params: {...}}; missing keys keep their defaults."""
    data = _load(path)
    cfg = RewardConfig()
    cfg.weights.update(data.get("weights", {}) or {})
    cfg.params.update(data.get("params", {}) or {})
    return cfg


def config_dir():
    """Installed config directory (share/social_nav_rl/config) if resolvable, else the source."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("social_nav_rl"), "config")
    except Exception:  # noqa: BLE001 - ament not sourced (e.g. bare pytest); use the source tree
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")


def load_experiment(cfg_dir=None):
    """Load the full set from a directory of YAMLs; returns a dict of the four configs."""
    d = cfg_dir or config_dir()
    return {
        "observation": load_obs_config(os.path.join(d, "observation.yaml")),
        "action": load_action_limits(os.path.join(d, "action.yaml")),
        "reward": load_reward_config(os.path.join(d, "reward.yaml")),
        "safety": load_safety_config(os.path.join(d, "safety.yaml")),
    }
