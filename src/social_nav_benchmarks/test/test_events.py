import json

from social_nav_benchmarks import events


def test_run_started_first_and_goal_reached(crossing_record, goal):
    evs = events.detect_events(crossing_record, goal)
    assert evs[0]["type"] == "RUN_STARTED"
    assert "GOAL_REACHED" in [e["type"] for e in evs]


def test_near_collision_and_intrusion_detected(crossing_record):
    evs = events.detect_events(crossing_record)
    types = [e["type"] for e in evs]
    assert "NEAR_COLLISION" in types
    assert "SOCIAL_INTRUSION" in types
    nc = next(e for e in evs if e["type"] == "NEAR_COLLISION")
    assert nc["severity"] == "critical"
    assert nc["min_human_dist"] < 0.5


def test_near_collision_emitted_once_per_episode(crossing_record):
    evs = events.detect_events(crossing_record)
    assert sum(1 for e in evs if e["type"] == "NEAR_COLLISION") == 1


def test_no_odom_flags_critical():
    evs = events.detect_events(
        {"scenario": "x", "run": 1, "status": "UNKNOWN", "odom": [], "humans": []})
    assert any(e["type"] == "NO_ODOM" and e["severity"] == "critical" for e in evs)


def test_failed_run_marks_run_failed():
    rec = {"scenario": "x", "run": 1, "status": "ABORTED",
           "odom": [(0.0, 0.0, 0.0, 0.0), (1.0, 0.1, 0.0, 0.1)], "humans": []}
    assert "RUN_FAILED" in [e["type"] for e in events.detect_events(rec)]


def test_events_sorted_by_time(crossing_record):
    ts = [e["t"] for e in events.detect_events(crossing_record)]
    assert ts == sorted(ts)


def test_write_events(tmp_path, crossing_record):
    path = tmp_path / "events.json"
    events.write_events(events.detect_events(crossing_record), str(path))
    data = json.loads(path.read_text())
    assert isinstance(data, list) and data
