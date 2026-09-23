from social_nav_rl.env import _wander


def test_zero_amplitude_is_noop():
    assert _wander(0, 3.7, 0.0) == (0.0, 0.0, 0.0, 0.0)


def test_deterministic():
    assert _wander(2, 1.5, 0.3) == _wander(2, 1.5, 0.3)


def test_perturbs_and_decorrelates_pedestrians():
    a = _wander(0, 2.0, 0.3)
    b = _wander(1, 2.0, 0.3)
    assert a != (0.0, 0.0, 0.0, 0.0) and a != b        # nonzero, and different peds differ


def test_velocity_matches_position_derivative():
    # dvx/dvy should approximate the finite-difference derivative of dx/dy
    t, h, amp = 2.0, 1e-4, 0.3
    dx0, dy0, dvx, dvy = _wander(3, t, amp)
    dx1, dy1, _, _ = _wander(3, t + h, amp)
    assert abs((dx1 - dx0) / h - dvx) < 1e-2 and abs((dy1 - dy0) / h - dvy) < 1e-2
