from social_nav_rl.action import ActionLimits, ActionMapper


def test_denormalize_endpoints():
    m = ActionMapper(ActionLimits(max_lin_vel=0.8, min_lin_vel=0.0, max_ang_vel=1.2))
    assert m.denormalize([-1.0, 0.0])[0] == 0.0     # min linear
    assert abs(m.denormalize([1.0, 0.0])[0] - 0.8) < 1e-9   # max linear
    assert abs(m.denormalize([0.0, 0.0])[0] - 0.4) < 1e-9   # midpoint
    assert abs(m.denormalize([0.0, 1.0])[1] - 1.2) < 1e-9   # max angular
    assert abs(m.denormalize([0.0, -1.0])[1] + 1.2) < 1e-9  # min angular


def test_clip_out_of_range():
    m = ActionMapper(ActionLimits(max_lin_vel=0.8))
    assert m.denormalize([5.0, 5.0])[0] == 0.8      # over-range clipped


def test_accel_clamp():
    m = ActionMapper(ActionLimits(max_lin_vel=1.0, max_lin_accel=1.0, dt=0.1))
    v, _ = m.denormalize([1.0, 0.0], prev_v=0.0, prev_w=0.0)
    assert abs(v - 0.1) < 1e-9      # target 1.0 but limited to prev + accel*dt = 0.1


def test_nonfinite_action_stops():
    m = ActionMapper(ActionLimits(min_lin_vel=0.0))
    v, w = m.denormalize([float("nan"), float("inf")])
    assert v == 0.0 and w == 0.0


def test_normalize_is_inverse():
    m = ActionMapper(ActionLimits(max_lin_vel=0.8, min_lin_vel=0.0, max_ang_vel=1.2))
    a0, a1 = m.normalize(0.4, 0.6)
    v, w = m.denormalize([a0, a1])
    assert abs(v - 0.4) < 1e-6 and abs(w - 0.6) < 1e-6
