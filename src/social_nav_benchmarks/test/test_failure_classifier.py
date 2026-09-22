from social_nav_benchmarks import failure_classifier as FC


def _row(**kw):
    base = {"scenario": "s", "run": 1, "success": False, "status": "UNKNOWN",
            "collision": False, "path_length_m": 5.0, "min_human_distance_m": 1.0,
            "oscillations": 0, "num_stops": 0, "social_intrusion_ratio": 0.0,
            "time_to_goal_s": None}
    base.update(kw)
    return base


def test_success_is_none():
    r = FC.classify(_row(success=True, status="SUCCEEDED", time_to_goal_s=18.0))
    assert r["label"] == "NONE"
    assert r["confidence"] == "high"


def test_collision_outranks_status():
    r = FC.classify(_row(success=False, status="SUCCEEDED", collision=True,
                         min_human_distance_m=0.3))
    assert r["label"] == "COLLISION"
    assert r["confidence"] == "high"


def test_timeout_label():
    assert FC.classify(_row(status="TIMEOUT", path_length_m=6.0))["label"] == "GOAL_TIMEOUT"


def test_stuck_when_barely_moved():
    r = FC.classify(_row(status="TIMEOUT", path_length_m=0.2))
    assert r["label"] == "STUCK"
    assert r["confidence"] == "medium"


def test_incomplete_is_low_confidence():
    r = FC.classify(_row(status="UNKNOWN", path_length_m=3.0))
    assert r["label"] == "INCOMPLETE"
    assert r["confidence"] == "low"


def test_quality_flags_on_success():
    r = FC.classify(_row(success=True, status="SUCCEEDED", oscillations=8,
                         num_stops=5, social_intrusion_ratio=0.6))
    assert set(r["flags"]) == {"OSCILLATION", "EXCESSIVE_STOPS", "SOCIAL_INTRUSION"}


def test_breakdown(sample_rows):
    b = FC.breakdown(sample_rows)
    assert b.get("NONE", 0) == 2
    assert b.get("STUCK", 0) == 1
