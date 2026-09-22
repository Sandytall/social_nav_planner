"""Pure-logic tests for per-planner comparison aggregation."""

import csv

from social_nav_benchmarks import comparison


def _rows():
    return [
        {"scenario": "empty", "run": 1, "planner": "social_nav", "success": True,
         "collision": False, "time_to_goal_s": 18.0, "min_human_distance_m": 1.5,
         "social_intrusion_ratio": 0.1, "compute_p95_ms": 0.12},
        {"scenario": "crossing", "run": 1, "planner": "social_nav", "success": True,
         "collision": False, "time_to_goal_s": 22.0, "min_human_distance_m": 0.9,
         "social_intrusion_ratio": 0.3, "compute_p95_ms": 0.15},
        {"scenario": "empty", "run": 1, "planner": "rpp", "success": True,
         "collision": False, "time_to_goal_s": 16.0, "min_human_distance_m": 1.2,
         "social_intrusion_ratio": 0.2},
        {"scenario": "crossing", "run": 1, "planner": "rpp", "success": False,
         "collision": True, "time_to_goal_s": None, "min_human_distance_m": 0.3,
         "social_intrusion_ratio": 0.6},
    ]


def test_planners_present_order_stable():
    assert comparison.planners_present(_rows()) == ["social_nav", "rpp"]


def test_planners_present_ignores_untagged():
    rows = [{"scenario": "x", "success": True}]  # no planner tag
    assert comparison.planners_present(rows) == []


def test_comparison_rows_aggregate():
    crows = {c["planner"]: c for c in comparison.comparison_rows(_rows())}
    assert crows["social_nav"]["runs"] == 2
    assert crows["social_nav"]["success_rate"] == 1.0
    assert crows["social_nav"]["collision_rate"] == 0.0
    # rpp: one success of two, one collision of two.
    assert crows["rpp"]["success_rate"] == 0.5
    assert crows["rpp"]["collision_rate"] == 0.5
    # worst clearance is the minimum across that planner's runs.
    assert crows["rpp"]["worst_clearance_m"] == 0.3
    # rpp published no latency -> n/a, never fabricated as 0.
    assert crows["rpp"]["mean_p95_ms"] is None


def test_write_comparison_csv(tmp_path):
    path = comparison.write_comparison_csv(_rows(), str(tmp_path / "comparison.csv"))
    with open(path) as f:
        table = list(csv.DictReader(f))
    assert {r["planner"] for r in table} == {"social_nav", "rpp"}
    assert table[0]["runs"] == "2"
