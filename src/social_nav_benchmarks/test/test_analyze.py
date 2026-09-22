import json
import os

from social_nav_benchmarks import analyze


def test_analyze_dir_end_to_end(tmp_path, sample_rows, crossing_record):
    d = str(tmp_path)
    for r in sample_rows:
        with open(os.path.join(d, f"{r['scenario']}_{r['run']:03d}.json"), "w") as f:
            json.dump(r, f)
    with open(os.path.join(d, "crossing_001_record.json"), "w") as f:
        json.dump(crossing_record, f)
    with open(os.path.join(d, "experiment_metadata.json"), "w") as f:
        json.dump({"timestamp_utc": "20260922-000000", "git_commit": "abc",
                   "ros_distro": "humble"}, f)

    summary = analyze.analyze_dir(d)

    assert summary["rows"] == 3
    assert summary["records_analyzed"] == 1
    for name in ("stats.json", "failures.json", "report.html",
                 "crossing_001_events.json", "crossing_001_trajectory.csv"):
        assert os.path.exists(os.path.join(d, name)), name


def test_analyze_dir_survives_no_records(tmp_path, sample_rows):
    d = str(tmp_path)
    for r in sample_rows:
        with open(os.path.join(d, f"{r['scenario']}_{r['run']:03d}.json"), "w") as f:
            json.dump(r, f)
    summary = analyze.analyze_dir(d)
    assert summary["records_analyzed"] == 0
    assert os.path.exists(os.path.join(d, "report.html"))
