import numpy as np

from social_nav_rl.observation import (
    HUMAN_FEATURES, ROBOT_FEATURES, Human, ObsConfig, ObservationAdapter, RobotState)


def _robot():
    return RobotState(x=0.0, y=0.0, yaw=0.0, v=0.3, w=0.0, goal_x=5.0, goal_y=0.0)


def test_size_and_empty():
    a = ObservationAdapter(ObsConfig(n_humans=5))
    assert a.size == ROBOT_FEATURES + 5 * HUMAN_FEATURES
    obs, mask = a.build(_robot(), [])
    assert obs.shape == (a.size,) and mask.sum() == 0 and np.all(np.isfinite(obs))


def test_nearest_n_selection_and_padding():
    a = ObservationAdapter(ObsConfig(n_humans=5, max_range=8.0))
    humans = [Human(id=i, x=float(i + 1), y=0.0) for i in range(6)]  # x = 1..6
    obs, mask = a.build(_robot(), humans)
    assert mask.sum() == 5                       # 6th (farthest) dropped, 5 kept
    assert abs(obs[ROBOT_FEATURES] - 1.0) < 1e-5  # first block is the nearest (x=1)


def test_padding_masks_unused_slots():
    a = ObservationAdapter(ObsConfig(n_humans=5))
    obs, mask = a.build(_robot(), [Human(id=0, x=2.0, y=0.0)])
    assert mask[0] == 1.0 and mask[1:].sum() == 0.0


def test_max_range_filter():
    a = ObservationAdapter(ObsConfig(n_humans=5, max_range=8.0))
    _, mask = a.build(_robot(), [Human(id=0, x=10.0, y=0.0)])
    assert mask.sum() == 0


def test_velocity_ablation_zeros_velocity_features():
    cfg = ObsConfig(n_humans=1, use_velocity=False)
    obs, _ = ObservationAdapter(cfg).build(_robot(), [Human(id=0, x=2.0, y=0.0, vx=1.0, vy=1.0)])
    # velocity features are indices 2,3 within the human block
    assert obs[ROBOT_FEATURES + 2] == 0.0 and obs[ROBOT_FEATURES + 3] == 0.0


def test_nan_human_is_sanitized():
    a = ObservationAdapter(ObsConfig(n_humans=1))
    obs, _ = a.build(_robot(), [Human(id=0, x=float("nan"), y=0.0)])
    assert np.all(np.isfinite(obs))


def test_deterministic():
    a = ObservationAdapter(ObsConfig(n_humans=3))
    h = [Human(id=0, x=2.0, y=1.0, vx=0.2, vy=0.0)]
    o1, _ = a.build(_robot(), h)
    o2, _ = a.build(_robot(), h)
    assert np.array_equal(o1, o2)
