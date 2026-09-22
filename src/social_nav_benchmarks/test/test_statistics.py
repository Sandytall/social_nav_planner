from social_nav_benchmarks import statistics as S


def test_summarize_basic():
    s = S.summarize([1, 2, 3, 4])
    assert s["count"] == 4
    assert s["min"] == 1 and s["max"] == 4
    assert abs(s["mean"] - 2.5) < 1e-6


def test_summarize_empty():
    s = S.summarize([])
    assert s["count"] == 0 and s["mean"] is None


def test_summarize_drops_none():
    assert S.summarize([1, None, 3])["count"] == 2


def test_aggregate_rates(sample_rows):
    a = S.aggregate(sample_rows)
    assert a["n"] == 3
    assert abs(a["success_rate"] - round(2 / 3, 3)) < 1e-9
    assert a["collision_rate"] == 0.0
    # empty-scenario row had no human distance, so only 2 values feed safety.
    assert a["dimensions"]["safety"]["min_human_distance_m"]["count"] == 2


def test_by_scenario(sample_rows):
    bs = S.by_scenario(sample_rows)
    assert set(bs.keys()) == {"empty", "crossing"}
    assert bs["crossing"]["n"] == 2
