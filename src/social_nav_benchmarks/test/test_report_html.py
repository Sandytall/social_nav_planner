import json
import os

from social_nav_benchmarks import report_html as R


def test_build_html_contains_sections(sample_rows):
    page = R.build_html(sample_rows, {"git_commit": "abc123", "ros_distro": "humble"})
    assert "<html" in page
    assert "SocialNav Benchmark Report" in page
    assert "crossing" in page and "empty" in page
    assert "measured" in page
    assert "abc123" in page


def test_generate_writes_file(tmp_path, sample_rows):
    for r in sample_rows:
        with open(tmp_path / f"{r['scenario']}_{r['run']:03d}.json", "w") as f:
            json.dump(r, f)
    out = R.generate(str(tmp_path))
    assert os.path.exists(out)
    assert "<html" in open(out).read()


def test_load_rows_skips_records_and_meta(tmp_path):
    (tmp_path / "empty_001.json").write_text(
        json.dumps({"scenario": "empty", "run": 1, "success": True, "status": "SUCCEEDED"}))
    (tmp_path / "empty_001_record.json").write_text(
        json.dumps({"scenario": "empty", "run": 1, "odom": []}))
    (tmp_path / "experiment_metadata.json").write_text(
        json.dumps({"timestamp_utc": "x", "scenarios": []}))
    rows = R.load_rows(str(tmp_path))
    assert len(rows) == 1 and rows[0]["scenario"] == "empty"
