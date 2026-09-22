"""Pure conversion tests for rosbag2 ingestion, using fake message objects (no ROS)."""

from types import SimpleNamespace as NS

from social_nav_benchmarks import bag_ingest


def _odom(x, y, vx, vy=0.0):
    return NS(pose=NS(pose=NS(position=NS(x=x, y=y))),
              twist=NS(twist=NS(linear=NS(x=vx, y=vy))))


def _humans(pairs):
    return NS(humans=[NS(pose=NS(position=NS(x=x, y=y))) for (x, y) in pairs])


def _metrics(p50, p95, p99, mx, misses, freq):
    return NS(p50_ms=p50, p95_ms=p95, p99_ms=p99, max_ms=mx,
              deadline_misses=misses, actual_frequency_hz=freq)


def test_odom_xyv_speed_is_magnitude():
    assert bag_ingest.odom_xyv(_odom(1.0, 2.0, 3.0, 4.0)) == (1.0, 2.0, 5.0)


def test_humans_xy():
    assert bag_ingest.humans_xy(_humans([(0.1, 0.2), (0.3, 0.4)])) == [(0.1, 0.2), (0.3, 0.4)]


def test_metrics_dict_types():
    d = bag_ingest.metrics_dict(_metrics(1, 2, 3, 4, 5, 20))
    assert d["p95_ms"] == 2.0 and d["deadline_misses"] == 5
    assert isinstance(d["deadline_misses"], int)


def test_build_record_normalizes_time_from_earliest_stamp():
    base = 1_000_000_000  # 1s in ns; earliest stamp across all topics
    odom_items = [(base, _odom(0.0, 0.0, 0.0)),
                  (base + 1_000_000_000, _odom(1.0, 1.0, 0.5))]
    human_items = [(base + 500_000_000, _humans([(2.0, 2.0)]))]
    metric_items = [(base + 250_000_000, _metrics(0.1, 0.2, 0.3, 0.5, 0, 20.0))]

    rec = bag_ingest.build_record("bag", 1, odom_items, human_items, metric_items)

    assert rec["scenario"] == "bag" and rec["source"] == "rosbag2"
    # First odom sample is at t=0 (earliest stamp), second at +1.0 s.
    assert rec["odom"][0][0] == 0.0
    assert abs(rec["odom"][1][0] - 1.0) < 1e-9
    assert rec["odom"][1][1:] == (1.0, 1.0, 0.5)
    # Human observation at +0.5 s.
    assert abs(rec["humans"][0][0] - 0.5) < 1e-9
    assert rec["humans"][0][1] == [(2.0, 2.0)]
    # planner keeps the last metrics dict; planner_series carries the stream.
    assert rec["planner"]["p95_ms"] == 0.2
    assert len(rec["planner_series"]) == 1


def test_build_record_without_metrics_leaves_planner_none():
    rec = bag_ingest.build_record("bag", 2, [(0, _odom(0.0, 0.0, 0.0))], [], [])
    assert rec["planner"] is None
    assert "planner_series" not in rec
