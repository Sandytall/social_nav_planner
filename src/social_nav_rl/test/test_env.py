import numpy as np
import pytest

pytest.importorskip("gymnasium")

from social_nav_rl.env import EpisodeConfig, SocialNavEnv          # noqa: E402
from social_nav_rl.observation import ObsConfig                    # noqa: E402


def _env(max_steps=50):
    return SocialNavEnv(EpisodeConfig(environment="urban", scenario="crossing",
                                      difficulty="easy", max_steps=max_steps),
                        ObsConfig(n_humans=5))


def test_spaces():
    e = _env()
    assert e.observation_space.shape[0] == e.adapter.size + 5
    assert e.action_space.shape == (2,)


def test_reset_shape_finite():
    e = _env()
    obs, info = e.reset(seed=1)
    assert obs.shape == e.observation_space.shape and np.all(np.isfinite(obs))


def test_step_contract():
    e = _env()
    e.reset(seed=1)
    obs, r, term, trunc, info = e.step(e.action_space.sample())
    assert obs.shape == e.observation_space.shape
    assert np.isscalar(r) and "reward_components" in info and "events" in info


def test_deterministic_seed():
    o1, _ = _env().reset(seed=7)
    o2, _ = _env().reset(seed=7)
    assert np.allclose(o1, o2)


def test_terminates_or_truncates():
    e = _env(max_steps=40)
    e.reset(seed=1)
    done = False
    for _ in range(40):
        _, _, term, trunc, _ = e.step(np.array([-1.0, 0.0], dtype=np.float32))
        done = term or trunc
        if done:
            break
    assert done


def test_nan_action_is_handled():
    e = _env()
    e.reset(seed=1)
    obs, r, _, _, info = e.step(np.array([np.nan, np.nan], dtype=np.float32))
    assert np.all(np.isfinite(obs)) and np.isfinite(r)
