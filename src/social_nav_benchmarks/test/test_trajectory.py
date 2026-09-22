from social_nav_benchmarks import trajectory


def test_rows_match_odom_length(crossing_record):
    rows = list(trajectory.rows(crossing_record))
    assert len(rows) == len(crossing_record["odom"])


def test_min_human_dist_computed(crossing_record):
    rows = list(trajectory.rows(crossing_record))
    row = next(r for r in rows if r["t"] == 3.0)
    assert abs(row["min_human_dist"] - 0.283) < 0.01
    assert row["num_humans"] == 1


def test_write_csv_header_and_length(tmp_path, crossing_record):
    path = tmp_path / "traj.csv"
    trajectory.write_trajectory_csv(crossing_record, str(path))
    lines = path.read_text().splitlines()
    assert lines[0] == ",".join(trajectory.FIELDS)
    assert len(lines) == 1 + len(crossing_record["odom"])
