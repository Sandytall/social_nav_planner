"""Render a self-contained HTML report from a benchmark results directory.

No plotting dependencies: overview tiles, a per-scenario table and success bars are drawn
with inline CSS and small inline SVG, so the report opens anywhere. Every number shown is
read from the recorded per-run rows; nothing is synthesised.

    social-nav-report --input ~/social_nav_results/<stamp>
"""

import argparse
import glob
import html
import json
import os

from social_nav_benchmarks import failure_classifier, statistics


def load_rows(results_dir):
    """Load per-run metric rows from a results directory.

    A file counts as a row if it carries scenario/run metrics and is not a raw record
    dump or the experiment-metadata file.
    """
    rows = []
    for path in sorted(glob.glob(os.path.join(results_dir, "*.json"))):
        if path.endswith("_record.json"):
            continue
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, ValueError):
            continue
        if not isinstance(data, dict):
            continue
        if "odom" in data or "timestamp_utc" in data or "scenarios" in data:
            continue
        if "scenario" in data and ("success" in data or "status" in data):
            rows.append(data)
    return rows


def load_meta(results_dir):
    path = os.path.join(results_dir, "experiment_metadata.json")
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _tile(label, value):
    return (f'<div class="tile"><div class="v">{html.escape(str(value))}</div>'
            f'<div class="l">{html.escape(label)}</div></div>')


def _fmt(value, suffix=""):
    if value is None:
        return "n/a"
    return f"{value}{suffix}"


def _success_bar(rate):
    pct = 0 if rate is None else int(round(rate * 100))
    return (f'<svg width="120" height="14" role="img" aria-label="{pct}% success">'
            f'<rect width="120" height="14" fill="#26303b"/>'
            f'<rect width="{pct * 1.2:.0f}" height="14" fill="#3fa15b"/></svg> {pct}%')


def build_html(rows, meta=None):
    """Return the report HTML as a string for the given rows."""
    meta = meta or {}
    agg = statistics.aggregate(rows)
    per_scn = statistics.by_scenario(rows)
    failures = failure_classifier.breakdown(rows)

    safety = agg["dimensions"]["safety"]["min_human_distance_m"]
    tgoal = agg["dimensions"]["efficiency"]["time_to_goal_s"]
    latency = agg["dimensions"]["system"]["compute_p95_ms"]

    tiles = "".join([
        _tile("runs", agg["n"]),
        _tile("success rate", _fmt(_pct(agg["success_rate"]), "%")),
        _tile("collision rate", _fmt(_pct(agg["collision_rate"]), "%")),
        _tile("mean time-to-goal", _fmt(tgoal["mean"], " s")),
        _tile("worst clearance", _fmt(safety["min"], " m")),
        _tile("p95 compute", _fmt(latency["mean"], " ms")),
    ])

    scn_rows = ""
    for name in sorted(per_scn):
        a = per_scn[name]
        eff = a["dimensions"]["efficiency"]["time_to_goal_s"]
        saf = a["dimensions"]["safety"]["min_human_distance_m"]
        soc = a["dimensions"]["social"]["social_intrusion_ratio"]
        scn_rows += (
            f"<tr><td>{html.escape(name)}</td>"
            f"<td>{_success_bar(a['success_rate'])}</td>"
            f"<td>{a['n']}</td>"
            f"<td>{_fmt(_pct(a['collision_rate']), '%')}</td>"
            f"<td>{_fmt(saf['min'], ' m')}</td>"
            f"<td>{_fmt(eff['mean'], ' s')}</td>"
            f"<td>{_fmt(soc['mean'])}</td></tr>")

    fail_rows = "".join(
        f"<tr><td>{html.escape(label)}</td><td>{count}</td></tr>"
        for label, count in sorted(failures.items(), key=lambda kv: -kv[1]))

    meta_line = " &middot; ".join(filter(None, [
        f"git {html.escape(str(meta.get('git_commit')))}" if meta.get("git_commit") else "",
        f"ROS {html.escape(str(meta.get('ros_distro')))}" if meta.get("ros_distro") else "",
        f"params {html.escape(str(meta.get('params_file')))}" if meta.get("params_file") else "",
        html.escape(str(meta.get("timestamp_utc"))) if meta.get("timestamp_utc") else "",
    ]))

    return _PAGE.format(
        meta_line=meta_line or "no experiment metadata found",
        tiles=tiles,
        scn_rows=scn_rows or '<tr><td colspan="7">no runs</td></tr>',
        fail_rows=fail_rows or '<tr><td colspan="2">no runs</td></tr>',
    )


def _pct(rate):
    return None if rate is None else round(rate * 100, 1)


def generate(results_dir, out_path=None, rows=None, meta=None):
    """Write the report HTML and return its path."""
    if rows is None:
        rows = load_rows(results_dir)
    if meta is None:
        meta = load_meta(results_dir)
    if out_path is None:
        out_path = os.path.join(results_dir, "report.html")
    with open(out_path, "w") as f:
        f.write(build_html(rows, meta))
    return out_path


def main():
    ap = argparse.ArgumentParser(description="Render an HTML report from a results dir")
    ap.add_argument("--input", required=True, help="benchmark results directory")
    ap.add_argument("--output", default=None, help="output HTML path (default report.html)")
    args = ap.parse_args()
    out = generate(args.input, args.output)
    print(f"Wrote {out}")


_PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SocialNav Benchmark Report</title>
<style>
  :root {{ color-scheme: dark; }}
  body {{ margin: 0; background: #11161c; color: #e6edf3;
         font: 15px/1.5 system-ui, sans-serif; padding: 24px; }}
  h1 {{ font-size: 22px; margin: 0 0 4px; }}
  .meta {{ color: #9aa7b4; font-size: 13px; margin-bottom: 20px; }}
  .tiles {{ display: flex; flex-wrap: wrap; gap: 12px; margin-bottom: 24px; }}
  .tile {{ background: #1a222c; border: 1px solid #26303b; border-radius: 8px;
          padding: 14px 18px; min-width: 120px; }}
  .tile .v {{ font-size: 22px; font-weight: 600; }}
  .tile .l {{ color: #9aa7b4; font-size: 12px; text-transform: uppercase;
             letter-spacing: .04em; }}
  h2 {{ font-size: 16px; margin: 24px 0 8px; }}
  table {{ border-collapse: collapse; width: 100%; max-width: 900px; }}
  th, td {{ text-align: left; padding: 7px 10px; border-bottom: 1px solid #26303b; }}
  th {{ color: #9aa7b4; font-weight: 600; font-size: 13px; }}
  footer {{ color: #6b7885; font-size: 12px; margin-top: 28px; }}
</style>
</head>
<body>
<h1>SocialNav Benchmark Report</h1>
<div class="meta">{meta_line}</div>
<div class="tiles">{tiles}</div>
<h2>Per scenario</h2>
<table>
<thead><tr><th>scenario</th><th>success</th><th>runs</th><th>collision</th>
<th>worst clearance</th><th>mean t-goal</th><th>intrusion ratio</th></tr></thead>
<tbody>{scn_rows}</tbody>
</table>
<h2>Failure breakdown</h2>
<table>
<thead><tr><th>label</th><th>runs</th></tr></thead>
<tbody>{fail_rows}</tbody>
</table>
<footer>Every value is measured from recorded runs; none are fabricated.
Safety, efficiency, social and system are reported separately, not blended.</footer>
</body>
</html>
"""


if __name__ == "__main__":
    main()
