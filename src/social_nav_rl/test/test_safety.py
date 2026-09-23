from social_nav_rl.safety import CAUTIOUS, NAN, NONE, SLOW, STOP, SafetyConfig, SafetySupervisor


def _sup():
    return SafetySupervisor(SafetyConfig())


def test_nonfinite_action_stops():
    d = _sup().filter(float("nan"), 0.0, {})
    assert d.v == 0.0 and d.w == 0.0 and d.kind == NAN and d.intervened


def test_imminent_ttc_emergency_stop():
    d = _sup().filter(0.8, 0.0, {"min_ttc": 0.5})
    assert d.v == 0.0 and d.kind == STOP


def test_close_clearance_emergency_stop():
    d = _sup().filter(0.8, 0.0, {"min_clearance": 0.2})
    assert d.v == 0.0 and d.kind == STOP


def test_stale_data_is_cautious():
    d = _sup().filter(0.8, 0.0, {"human_data_age": 5.0})
    assert d.kind == CAUTIOUS and d.v <= SafetyConfig().cautious_speed + 1e-9


def test_moderate_ttc_slows():
    d = _sup().filter(0.8, 0.0, {"min_ttc": 2.0})   # between ttc_stop(1) and ttc_slow(3)
    assert d.kind == SLOW and 0.0 < d.v < 0.8


def test_clear_path_no_intervention_but_limited():
    d = _sup().filter(5.0, 5.0, {"min_ttc": 100.0, "min_clearance": 5.0})
    assert d.kind == NONE and not d.intervened
    assert d.v <= 0.8 and abs(d.w) <= 1.2          # clamped to limits


def test_intervention_log_counts():
    s = _sup()
    s.filter(float("nan"), 0.0, {})
    s.filter(0.8, 0.0, {"min_ttc": 0.5})
    counts = s.intervention_counts()
    assert counts.get(NAN) == 1 and counts.get(STOP) == 1
