import types

import pytest

pytest.importorskip("gymnasium")

from social_nav_rl import ablation                    # noqa: E402
from social_nav_rl import config as C                 # noqa: E402
from social_nav_rl.policies import StraightToGoalPolicy  # noqa: E402


def _args():
    return types.SimpleNamespace(environment="urban", scenario="crossing", difficulty="easy",
                                 split="test", episodes=1, max_steps=40)


def test_run_ablation_full():
    exp = C.load_experiment()
    r = ablation.run_ablation("full", exp["observation"], exp, _args(), StraightToGoalPolicy())
    assert 0.0 <= r["success_rate"] <= 1.0 and r["obs_dim"] > 0 and "diagnostics" in r


def test_no_humans_has_smaller_obs():
    exp = C.load_experiment()
    full = ablation.run_ablation("full", exp["observation"], exp, _args(), StraightToGoalPolicy())
    none = ablation.run_ablation("no_humans", exp["observation"], exp, _args(), StraightToGoalPolicy())
    assert none["obs_dim"] < full["obs_dim"]


def test_ablation_names_map_to_obsconfig():
    from social_nav_rl.observation import ObsConfig
    for name, overrides in ablation.ABLATIONS.items():
        cfg = ablation._obs_config(ObsConfig(), overrides)
        assert cfg.n_humans >= 0
