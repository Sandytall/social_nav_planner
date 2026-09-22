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

from social_nav_benchmarks import comparison, common, failure_classifier, statistics
from social_nav_benchmarks.metrics import COLLISION_DIST, COMFORT_DIST


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


# --------------------------------------------------------------------------------------
# Inline-SVG charts (no plotting libraries; every value comes from the recorded series).
# --------------------------------------------------------------------------------------

_PAD = 6


def _scale(v, lo, hi, size, invert=False):
    if hi - lo < 1e-9:
        frac = 0.5
    else:
        frac = (v - lo) / (hi - lo)
    frac = max(0.0, min(1.0, frac))
    span = size - 2 * _PAD
    return _PAD + (span * (1 - frac) if invert else span * frac)


def _line_chart(series, width, height, color, aria, refs=None, ymin=0.0, ymax=None):
    """A time-series line. `series` is [(t, y)]; refs is [(y, color)] horizontal markers."""
    pts = [(t, y) for (t, y) in series if y is not None]
    if len(pts) < 2:
        return f'<svg class="chart" width="{width}" height="{height}" role="img" ' \
               f'aria-label="{html.escape(aria)} (insufficient data)"></svg>'
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    xlo, xhi = min(xs), max(xs)
    ylo = ymin if ymin is not None else min(ys)
    yhi = ymax if ymax is not None else max(ys)
    if yhi <= ylo:
        yhi = ylo + 1.0
    coords = " ".join(
        f"{_scale(x, xlo, xhi, width):.1f},{_scale(y, ylo, yhi, height, invert=True):.1f}"
        for (x, y) in pts)
    ref_svg = ""
    for (ry, rc) in (refs or []):
        if ylo <= ry <= yhi:
            py = _scale(ry, ylo, yhi, height, invert=True)
            ref_svg += (f'<line x1="{_PAD}" y1="{py:.1f}" x2="{width - _PAD}" '
                        f'y2="{py:.1f}" stroke="{rc}" stroke-width="1" '
                        f'stroke-dasharray="3 3" opacity="0.7"/>')
    return (f'<svg class="chart" width="{width}" height="{height}" role="img" '
            f'aria-label="{html.escape(aria)}">'
            f'<rect width="{width}" height="{height}" fill="#0e1319"/>'
            f'{ref_svg}'
            f'<polyline fill="none" stroke="{color}" stroke-width="1.5" '
            f'points="{coords}"/></svg>')


def _path_sparkline(points, width, height, color, aria):
    """A robot x-y path drawn with a uniform scale (preserves shape), y up."""
    pts = [(x, y) for (x, y) in points if x is not None and y is not None]
    if len(pts) < 2:
        return f'<svg class="chart" width="{width}" height="{height}" role="img" ' \
               f'aria-label="{html.escape(aria)} (insufficient data)"></svg>'
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    xlo, xhi, ylo, yhi = min(xs), max(xs), min(ys), max(ys)
    span = max(xhi - xlo, yhi - ylo, 1e-6)
    inner = min(width, height) - 2 * _PAD
    ox = (width - inner) / 2
    oy = (height - inner) / 2

    def px(x):
        return ox + _PAD + inner * (x - xlo) / span

    def py(y):
        return oy + _PAD + inner * (1 - (y - ylo) / span)

    coords = " ".join(f"{px(x):.1f},{py(y):.1f}" for (x, y) in pts)
    sx, sy = pts[0]
    ex, ey = pts[-1]
    return (f'<svg class="chart" width="{width}" height="{height}" role="img" '
            f'aria-label="{html.escape(aria)}">'
            f'<rect width="{width}" height="{height}" fill="#0e1319"/>'
            f'<polyline fill="none" stroke="{color}" stroke-width="1.5" '
            f'points="{coords}"/>'
            f'<circle cx="{px(sx):.1f}" cy="{py(sy):.1f}" r="2.5" fill="#3fa15b"/>'
            f'<circle cx="{px(ex):.1f}" cy="{py(ey):.1f}" r="2.5" fill="#d08770"/></svg>')


def _bar_chart(pairs, width, height, color, aria):
    """Small labelled vertical bars. `pairs` is [(label, value)]; None values are skipped."""
    vals = [(lab, v) for (lab, v) in pairs if isinstance(v, (int, float))]
    if not vals:
        return f'<svg class="chart" width="{width}" height="{height}" role="img" ' \
               f'aria-label="{html.escape(aria)} (no data)"></svg>'
    vmax = max(v for _, v in vals) or 1.0
    n = len(vals)
    bw = (width - 2 * _PAD) / n
    bars = ""
    for i, (lab, v) in enumerate(vals):
        bh = (height - 2 * _PAD - 8) * (v / vmax)
        x = _PAD + i * bw
        y = height - _PAD - 8 - bh
        bars += (f'<rect x="{x + 2:.1f}" y="{y:.1f}" width="{bw - 4:.1f}" '
                 f'height="{bh:.1f}" fill="{color}"/>'
                 f'<text x="{x + bw / 2:.1f}" y="{height - 1}" fill="#9aa7b4" '
                 f'font-size="8" text-anchor="middle">{html.escape(str(lab))}</text>')
    return (f'<svg class="chart" width="{width}" height="{height}" role="img" '
            f'aria-label="{html.escape(aria)}">'
            f'<rect width="{width}" height="{height}" fill="#0e1319"/>{bars}</svg>')


def load_details(results_dir):
    """Per-run series for the charts, read from the raw ``*_record.json`` dumps.

    Everything is derived from recorded signals: min-human-distance and robot path from the
    odom/human streams, and the compute-latency series from the controller's PlannerMetrics
    stream when it was published (baselines do not publish it, so their series is empty and
    the report falls back to the aggregate percentiles). Returns [] if no records exist.
    """
    details = []
    for path in sorted(glob.glob(os.path.join(results_dir, "*_record.json"))):
        try:
            with open(path) as f:
                rec = json.load(f)
        except (OSError, ValueError):
            continue
        dist_series = [(t, d) for (t, _xy, _v, d) in common.clearance_series(rec)]
        odom = rec.get("odom") or []
        speed_series = [(t, v) for (t, _x, _y, v) in odom]
        path_xy = [(x, y) for (_t, x, y, _v) in odom]
        latency_series = [(s[0], s[2]) for s in rec.get("planner_series") or []]  # (t, p95)
        pm = rec.get("planner") or {}
        latency_pcts = [
            ("p50", pm.get("p50_ms")), ("p95", pm.get("p95_ms")),
            ("p99", pm.get("p99_ms")), ("max", pm.get("max_ms")),
        ]
        details.append({
            "scenario": rec.get("scenario", "?"),
            "run": rec.get("run"),
            "planner": rec.get("planner_name"),
            "dist_series": dist_series,
            "speed_series": speed_series,
            "path": path_xy,
            "latency_series": latency_series,
            "latency_pcts": latency_pcts,
            "telemetry": rec.get("telemetry"),
        })
    return details


def _fmt_num(value, suffix=""):
    if value is None:
        return "n/a"
    return f"{value}{suffix}"


def _comparison_section(rows):
    """A planner-comparison table plus success bars; empty unless >1 planner is present."""
    planners = comparison.planners_present(rows)
    if len(planners) < 2:
        return ""
    crows = comparison.comparison_rows(rows)
    crows = [c for c in crows if c["planner"] is not None]
    head = ("<tr><th>planner</th><th>runs</th><th>success</th><th>collision</th>"
            "<th>mean t-goal</th><th>worst clear</th><th>mean clear</th>"
            "<th>intrusion</th><th>p95 ms</th></tr>")
    body = ""
    for c in crows:
        body += (
            f"<tr><td>{html.escape(str(c['planner']))}</td>"
            f"<td>{c['runs']}</td>"
            f"<td>{_success_bar(c['success_rate'])}</td>"
            f"<td>{_fmt_num(_pct(c['collision_rate']), '%')}</td>"
            f"<td>{_fmt_num(c['mean_time_to_goal_s'], ' s')}</td>"
            f"<td>{_fmt_num(c['worst_clearance_m'], ' m')}</td>"
            f"<td>{_fmt_num(c['mean_min_clearance_m'], ' m')}</td>"
            f"<td>{_fmt_num(c['mean_intrusion_ratio'])}</td>"
            f"<td>{_fmt_num(c['mean_p95_ms'], ' ms')}</td></tr>")
    return ('<h2>Planner comparison</h2>'
            '<p class="note">Same scenarios, seeds and goal per planner; only the '
            'local FollowPath controller differs. Compute latency is n/a for baselines '
            'that do not publish PlannerMetrics.</p>'
            f'<table><thead>{head}</thead><tbody>{body}</tbody></table>')


def _per_run_section(details):
    """A card per run with min-distance, speed, path and latency charts (all measured)."""
    if not details:
        return ""
    cards = ""
    for d in sorted(details, key=lambda x: (str(x.get("planner")), x["scenario"],
                                            x.get("run") or 0)):
        tag = f"{html.escape(str(d['planner']))} &middot; " if d.get("planner") else ""
        title = f"{tag}{html.escape(d['scenario'])} run {d.get('run')}"
        if d["latency_series"]:
            latency = _line_chart(d["latency_series"], 150, 70, "#b48ead",
                                  "controller p95 latency over time (ms)")
            lat_label = "p95 latency over time (ms)"
        else:
            latency = _bar_chart(d.get("latency_pcts") or [], 150, 70, "#b48ead",
                                 "compute latency percentiles (ms)")
            lat_label = "latency percentiles (ms)"
        tel = d.get("telemetry") or {}
        cap = _telemetry_caption(tel)
        dist = _line_chart(d["dist_series"], 150, 70, "#88c0d0",
                           "min human distance over time (m)",
                           refs=[(COLLISION_DIST, "#bf616a"), (COMFORT_DIST, "#ebcb8b")])
        speed = _line_chart(d["speed_series"], 150, 70, "#a3be8c",
                            "robot speed over time (m/s)")
        path = _path_sparkline(d["path"], 90, 70, "#81a1c1", "robot trajectory x-y")
        cards += (
            '<div class="card">'
            f'<div class="ct">{title}</div>'
            '<div class="charts">'
            f'<figure>{dist}<figcaption>min human dist (m)</figcaption></figure>'
            f'<figure>{speed}<figcaption>robot speed (m/s)</figcaption></figure>'
            f'<figure>{path}<figcaption>trajectory</figcaption></figure>'
            f'<figure>{latency}<figcaption>{lat_label}</figcaption></figure>'
            '</div>'
            f'{cap}'
            '</div>')
    return ('<h2>Per-run detail</h2>'
            '<p class="note">Distance and speed are time series from the recorded run; '
            'the trajectory dot markers are start (green) and end (orange). Dashed lines '
            f'mark the collision ({COLLISION_DIST} m) and comfort ({COMFORT_DIST} m) '
            'thresholds.</p>'
            f'<div class="cards">{cards}</div>')


def _telemetry_caption(tel):
    if not tel:
        return ""
    parts = []
    cpu = tel.get("cpu_percent")
    if cpu:
        parts.append(f"CPU peak {cpu.get('peak')}%")
    ram = tel.get("ram_used_mb")
    if ram:
        parts.append(f"RAM peak {ram.get('peak')} MB")
    gpu = tel.get("gpu") or {}
    if gpu.get("available") and gpu.get("util_percent"):
        parts.append(f"GPU peak {gpu['util_percent'].get('peak')}%")
    elif gpu and not gpu.get("available"):
        parts.append("GPU n/a")
    rtf = tel.get("rtf")
    parts.append(f"RTF {rtf}" if rtf is not None else "RTF n/a")
    if not parts:
        return ""
    body = " &middot; ".join(html.escape(p) for p in parts)
    return f'<div class="tel">{body}</div>'


def build_html(rows, meta=None, details=None):
    """Return the report HTML as a string for the given rows.

    `details` (per-run series from load_details) adds the per-run charts section; when it is
    None only the summary tables are rendered, so the pure unit tests stay ROS-free.
    """
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
        comparison=_comparison_section(rows),
        scn_rows=scn_rows or '<tr><td colspan="7">no runs</td></tr>',
        fail_rows=fail_rows or '<tr><td colspan="2">no runs</td></tr>',
        per_run=_per_run_section(details),
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
    details = load_details(results_dir)
    with open(out_path, "w") as f:
        f.write(build_html(rows, meta, details=details))
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
  .note {{ color: #9aa7b4; font-size: 12px; max-width: 760px; margin: 4px 0 10px; }}
  .cards {{ display: flex; flex-wrap: wrap; gap: 12px; }}
  .card {{ background: #1a222c; border: 1px solid #26303b; border-radius: 8px;
          padding: 10px 12px; }}
  .card .ct {{ font-size: 13px; font-weight: 600; margin-bottom: 6px; }}
  .charts {{ display: flex; flex-wrap: wrap; gap: 8px; align-items: flex-end; }}
  .charts figure {{ margin: 0; }}
  .charts figcaption {{ color: #9aa7b4; font-size: 10px; text-align: center;
                       margin-top: 2px; }}
  svg.chart {{ border: 1px solid #26303b; border-radius: 4px; display: block; }}
  .tel {{ color: #9aa7b4; font-size: 11px; margin-top: 6px; }}
  footer {{ color: #6b7885; font-size: 12px; margin-top: 28px; }}
</style>
</head>
<body>
<h1>SocialNav Benchmark Report</h1>
<div class="meta">{meta_line}</div>
<div class="tiles">{tiles}</div>
{comparison}
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
{per_run}
<footer>Every value is measured from recorded runs; none are fabricated.
Safety, efficiency, social and system are reported separately, not blended.</footer>
</body>
</html>
"""


if __name__ == "__main__":
    main()
