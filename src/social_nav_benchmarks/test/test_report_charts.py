"""Tests for the inline-SVG charts and the comparison / per-run report sections."""

import json

from social_nav_benchmarks import report_html as R


def test_line_chart_draws_polyline():
    svg = R._line_chart([(0, 1.0), (1, 0.5), (2, 0.8)], 100, 50, "#fff", "d")
    assert "<svg" in svg and "<polyline" in svg


def test_line_chart_insufficient_data_no_polyline():
    svg = R._line_chart([(0, 1.0)], 100, 50, "#fff", "d")
    assert "<svg" in svg and "<polyline" not in svg


def test_line_chart_reference_lines():
    svg = R._line_chart([(0, 0.2), (1, 0.4)], 100, 50, "#fff", "d",
                        refs=[(0.3, "#bf616a")], ymin=0.0, ymax=1.0)
    assert "<line" in svg  # the reference threshold


def test_path_sparkline():
    svg = R._path_sparkline([(0, 0), (1, 1), (2, 0)], 80, 60, "#fff", "path")
    assert "<polyline" in svg and svg.count("<circle") == 2  # start + end markers


def test_bar_chart_skips_none():
    svg = R._bar_chart([("p50", None), ("p95", 0.2)], 100, 50, "#fff", "lat")
    assert svg.count("<rect") >= 2  # background + one bar


def test_bar_chart_all_none_no_bars():
    svg = R._bar_chart([("p50", None)], 100, 50, "#fff", "lat")
    assert "no data" in svg


def _two_planner_rows():
    return [
        {"scenario": "empty", "run": 1, "planner": "social_nav", "success": True,
         "collision": False, "time_to_goal_s": 18.0, "min_human_distance_m": 1.5,
         "social_intrusion_ratio": 0.1, "compute_p95_ms": 0.12},
        {"scenario": "empty", "run": 1, "planner": "rpp", "success": True,
         "collision": False, "time_to_goal_s": 16.0, "min_human_distance_m": 1.1,
         "social_intrusion_ratio": 0.2},
    ]


def test_comparison_section_present_with_two_planners():
    html = R._comparison_section(_two_planner_rows())
    assert "Planner comparison" in html
    assert "social_nav" in html and "rpp" in html


def test_comparison_section_empty_with_single_planner(sample_rows):
    # sample_rows carry no planner tag -> no comparison section.
    assert R._comparison_section(sample_rows) == ""


def test_build_html_with_details_and_planners():
    details = [{
        "scenario": "empty", "run": 1, "planner": "social_nav",
        "dist_series": [(0, 1.5), (1, 1.2)], "speed_series": [(0, 0.0), (1, 0.5)],
        "path": [(0, 0), (1, 1)], "latency_series": [], "latency_pcts": [("p95", 0.12)],
        "telemetry": {"cpu_percent": {"mean": 20, "peak": 40}, "rtf": 0.9,
                      "gpu": {"available": False}},
    }]
    page = R.build_html(_two_planner_rows(), {"git_commit": "abc"}, details=details)
    assert "Planner comparison" in page
    assert "Per-run detail" in page
    assert "<svg" in page
    assert "RTF 0.9" in page and "GPU n/a" in page


def test_load_details_from_record(tmp_path):
    record = {
        "scenario": "crossing", "run": 1, "planner_name": "social_nav",
        "odom": [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 1.0, 0.5)],
        "humans": [(0.0, [(0.5, 0.5)]), (1.0, [(2.0, 2.0)])],
        "planner": {"p50_ms": 0.08, "p95_ms": 0.12, "p99_ms": 0.2, "max_ms": 0.4},
        "planner_series": [(0.0, 0.08, 0.12, 0.2, 0.4, 20.0, 0)],
        "telemetry": {"rtf": 0.8},
    }
    (tmp_path / "crossing_001_record.json").write_text(json.dumps(record))
    details = R.load_details(str(tmp_path))
    assert len(details) == 1
    d = details[0]
    assert d["planner"] == "social_nav"
    assert len(d["dist_series"]) == 2          # one per human observation
    assert len(d["speed_series"]) == 2         # one per odom sample
    assert d["latency_series"] == [(0.0, 0.12)]
