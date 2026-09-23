#!/usr/bin/env python3
"""System identification: measure the mock->Gazebo dynamics/sensor gap, then set DR ranges.

Item 3 of the transfer plan. Domain randomization (social_nav_rl/randomize.py, `--domain-rand`)
only helps if its ranges bracket the REAL gap between the accelerated MockBackend (perfect,
instant velocity tracking; clean ray-cast lidar) and Gazebo (laggy, imperfect tracking; noisy
/scan). This script drives an IDENTICAL open-loop command program through either backend, logs the
response, and fits the gap into concrete DR numbers you can paste into `default_profile()`.

Usage
-----
  # 1. baseline in the mock (headless, instant) - should show ~zero deviation:
  python3 scripts/sysid.py record --backend mock --out /tmp/sysid_mock.json

  # 2. the real system (needs the RL sim up WITHOUT the Nav2 controller):
  #    ros2 launch social_nav_bringup rl_sim.launch.py environment:=factory scenario:=normal
  python3 scripts/sysid.py record --backend gazebo --out /tmp/sysid_gazebo.json

  # 3. fit the gap and print suggested DR ranges:
  python3 scripts/sysid.py analyze /tmp/sysid_gazebo.json /tmp/sysid_mock.json

The command program is open-loop (no policy), so it is safe and repeatable. Nothing here writes to
the repo or trains; it only measures.
"""
import argparse
import json
import math
import statistics
import sys

DT = 0.1  # control period (matches ActionLimits.dt); keep both backends on the same dt

# (label, linear v, angular w, seconds). Chosen to expose: a stationary baseline (lidar noise), a
# forward step (latency + gain), a mid-speed hold (steady gain + tracking noise), a pure turn
# (yaw-rate gain) and a combined arc.
PROGRAM = [
    ("settle",   0.00, 0.00, 1.5),
    ("fwd_step", 0.60, 0.00, 2.0),
    ("stop1",    0.00, 0.00, 1.0),
    ("fwd_mid",  0.35, 0.00, 2.0),
    ("stop2",    0.00, 0.00, 1.0),
    ("turn",     0.00, 0.60, 2.0),
    ("stop3",    0.00, 0.00, 1.0),
    ("arc",      0.40, 0.40, 2.0),
    ("stop4",    0.00, 0.00, 1.0),
]


def steps(dt=DT):
    """Flatten PROGRAM into per-control-step (label, v, w)."""
    out = []
    for label, v, w, secs in PROGRAM:
        for _ in range(max(1, int(round(secs / dt)))):
            out.append((label, v, w))
    return out


# --------------------------------------------------------------------------- record: mock
def record_mock(out_path, environment, scenario, difficulty, dt=DT):
    """Drive the MockBackend open-loop and log its (perfect) response + clean ray-cast lidar."""
    from social_nav_rl.env import EpisodeConfig, MockBackend
    from social_nav_rl.observation import ObsConfig

    obs_cfg = ObsConfig()
    ep = EpisodeConfig(environment=environment, scenario=scenario, difficulty=difficulty)
    be = MockBackend(ep, dt, obs_cfg=obs_cfg)
    be.reset(seed=0)
    log = []
    t = 0.0
    for label, v, w in steps(dt):
        be.step(v, w)                       # mock integrates exactly -> exec == cmd
        r = be.robot
        lidar = be.lidar()
        log.append({"t": round(t, 3), "seg": label, "cmd_v": v, "cmd_w": w,
                    "exec_v": r["v"], "exec_w": r["w"], "x": r["x"], "y": r["y"], "yaw": r["yaw"],
                    "beams": [float(b) for b in lidar] if lidar is not None else None})
        t += dt
    _dump(out_path, {"backend": "mock", "dt": dt, "n_beams": obs_cfg.n_lidar,
                     "lidar_range": obs_cfg.lidar_range, "log": log})


# --------------------------------------------------------------------------- record: gazebo
def record_gazebo(out_path, dt=DT, n_beams=12, lidar_range=5.0):
    """Drive the real robot open-loop over /cmd_vel and log odom twist + down-sampled /scan."""
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import LaserScan
    from social_nav_rl.perception import downsample_scan

    if not rclpy.ok():
        rclpy.init()
    node = Node("social_nav_sysid")
    if not node.has_parameter("use_sim_time"):
        node.declare_parameter("use_sim_time", True)
    else:
        node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

    state = {"odom": None, "scan": None}
    node.create_subscription(Odometry, "odom", lambda m: state.__setitem__("odom", m), 20)
    node.create_subscription(LaserScan, "scan", lambda m: state.__setitem__("scan", m), 10)
    cmd = node.create_publisher(Twist, "cmd_vel", 10)

    def spin(secs):
        clk = node.get_clock()
        t0 = clk.now()
        while (clk.now() - t0).nanoseconds * 1e-9 < secs:
            rclpy.spin_once(node, timeout_sec=0.02)

    node.get_logger().info("waiting for odom + scan...")
    for _ in range(200):
        spin(0.05)
        if state["odom"] is not None and state["scan"] is not None:
            break
    if state["odom"] is None:
        node.get_logger().error("no /odom - is the RL sim running? aborting.")
        return

    log, t = [], 0.0
    for label, v, w in steps(dt):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        cmd.publish(m)
        spin(dt)
        od, sc = state["odom"], state["scan"]
        beams, raw_invalid = None, None
        if sc is not None:
            beams = [float(b) for b in downsample_scan(list(sc.ranges), sc.angle_min,
                     sc.angle_increment, n_beams=n_beams, max_range=lidar_range)]
            # True dropout must come from the RAW scan: after down-sampling, a clear beam and a
            # missed-return beam both read max_range and are indistinguishable.
            bad = sum(1 for r in sc.ranges if not math.isfinite(r) or r <= 0.0)
            raw_invalid = bad / max(1, len(sc.ranges))
        tw = od.twist.twist
        p = od.pose.pose
        log.append({"t": round(t, 3), "seg": label, "cmd_v": v, "cmd_w": w,
                    "exec_v": tw.linear.x, "exec_w": tw.angular.z,
                    "x": p.position.x, "y": p.position.y, "yaw": _yaw(p.orientation),
                    "beams": beams, "raw_invalid": raw_invalid})
        t += dt
    cmd.publish(Twist())                    # stop the robot
    spin(0.3)
    _dump(out_path, {"backend": "gazebo", "dt": dt, "n_beams": n_beams,
                     "lidar_range": lidar_range, "log": log})
    node.destroy_node()


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# --------------------------------------------------------------------------- analyze
def _seg(log, name):
    return [r for r in log if r["seg"] == name]


def _steady(rows, key, skip=5):
    """Values of `key` over a segment after dropping the first `skip` transient samples."""
    return [r[key] for r in rows[skip:]] if len(rows) > skip else [r[key] for r in rows]


def analyze(gazebo_path, mock_path=None):
    data = _load(gazebo_path)
    log, dt = data["log"], data.get("dt", DT)
    print(f"# system-ID from {gazebo_path}  (backend={data['backend']}, dt={dt})\n")

    # velocity gain + tracking noise (steady portion of the mid-speed hold)
    mid = _seg(log, "fwd_mid")
    cmd_v = mid[0]["cmd_v"] if mid else 0.35
    ev = _steady(mid, "exec_v")
    gain_v = (statistics.median(ev) / cmd_v) if ev and cmd_v else float("nan")
    track = (statistics.pstdev(ev) / abs(statistics.mean(ev))) if len(ev) > 1 and statistics.mean(ev) else 0.0

    # yaw-rate gain (steady portion of the pure turn)
    turn = _seg(log, "turn")
    cmd_w = turn[0]["cmd_w"] if turn else 0.6
    ew = _steady(turn, "exec_w")
    gain_w = (statistics.median(ew) / cmd_w) if ew and cmd_w else float("nan")

    # latency: on the forward step, first time exec_v crosses 63% of its steady value
    step = _seg(log, "fwd_step")
    lat_s = _rise_time(step, "exec_v", dt)

    # lidar noise + dropout from the stationary settle segment (per-beam temporal spread)
    noise, dropout = _lidar_stats(_seg(log, "settle"), data.get("lidar_range", 5.0))

    print(f"velocity gain   (exec/cmd @ {cmd_v} m/s) : {gain_v:.3f}"
          f"   -> speed_scale center |1-gain| = {abs(1 - gain_v):.3f}")
    print(f"yaw-rate gain   (exec/cmd @ {cmd_w} rad/s): {gain_w:.3f}")
    print(f"tracking noise  (rel. stddev, steady)     : {track:.3f}")
    print(f"control latency (63% rise on step)        : {lat_s:.2f} s  = {lat_s / dt:.1f} steps")
    if noise is not None:
        print(f"lidar noise     (per-beam stddev, still)  : {noise:.3f} m")
        print(f"lidar dropout   (invalid-return frac)     : {dropout:.3f}")

    if mock_path:
        mlog = _load(mock_path)["log"]
        mev = _steady(_seg(mlog, "fwd_mid"), "exec_v")
        mgain = (statistics.median(mev) / cmd_v) if mev and cmd_v else float("nan")
        print(f"\n(mock baseline velocity gain: {mgain:.3f} - expected ~1.000, confirms the rig)")

    # suggested DR profile: bracket each measured gap with a margin
    print("\n# suggested randomize.default_profile() from these measurements:")
    print("DRConfig(enabled=True,")
    if noise is not None:
        print(f"         lidar_noise={_r(max(noise, 0.01))}, lidar_dropout={_r(max(dropout, 0.0))},")
    print(f"         action_latency={max(1, round(lat_s / dt))},")
    print(f"         vel_tracking_err={_r(max(track * 1.5, 0.05))},")
    print(f"         speed_scale={_r(max(abs(1 - gain_v) + 0.05, 0.05))},")
    print("         human_speed_scale=0.2)   # crowd speed isn't rig-measurable; keep a spread")


def _rise_time(rows, key, dt):
    if not rows:
        return 0.0
    final = statistics.median([r[key] for r in rows[len(rows) // 2:]]) or 1e-6
    for i, r in enumerate(rows):
        if abs(r[key]) >= 0.63 * abs(final):
            return i * dt
    return len(rows) * dt


def _lidar_stats(rows, max_range):
    """Return (per-beam temporal noise stddev [m], dropout fraction) from a stationary segment.

    Noise is the spread of beams that see an obstacle (finite, >0, < max_range); a beam at
    max_range is *clear*, not noisy, so it is excluded. Dropout is the fraction of genuinely
    missed returns - taken from the raw-scan `raw_invalid` the recorder stored, because after
    down-sampling a clear beam and a missed one both read max_range and can't be told apart.
    """
    beams = [r["beams"] for r in rows if r.get("beams")]
    if not beams:
        return None, None
    n = min(len(b) for b in beams)
    stds = []
    for j in range(n):
        col = [b[j] for b in beams
               if math.isfinite(b[j]) and 0.0 < b[j] < max_range]   # obstacle-seeing beams only
        if len(col) > 1:
            stds.append(statistics.pstdev(col))
    noise = statistics.mean(stds) if stds else 0.0
    raw = [r["raw_invalid"] for r in rows if r.get("raw_invalid") is not None]
    dropout = statistics.mean(raw) if raw else 0.0                  # 0.0 for the mock (no raw scan)
    return noise, dropout


def _r(x):
    return round(float(x), 3)


def _dump(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)
    print(f"wrote {path}  ({len(obj['log'])} samples, backend={obj['backend']})")


def _load(path):
    with open(path) as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    rec = sub.add_parser("record", help="drive one backend open-loop and log the response")
    rec.add_argument("--backend", required=True, choices=["mock", "gazebo"])
    rec.add_argument("--out", required=True)
    rec.add_argument("--environment", default="factory")
    rec.add_argument("--scenario", default="normal")
    rec.add_argument("--difficulty", default="medium")

    an = sub.add_parser("analyze", help="fit the gap and print suggested DR ranges")
    an.add_argument("gazebo_log")
    an.add_argument("mock_log", nargs="?", default=None)

    args = ap.parse_args()
    if args.cmd == "record":
        if args.backend == "mock":
            record_mock(args.out, args.environment, args.scenario, args.difficulty)
        else:
            record_gazebo(args.out)
    elif args.cmd == "analyze":
        analyze(args.gazebo_log, args.mock_log)


if __name__ == "__main__":
    sys.exit(main())
