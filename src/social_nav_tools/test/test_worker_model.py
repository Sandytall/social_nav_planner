"""Unit tests for the deterministic factory-worker planning logic (no ROS/Gazebo needed)."""
import math

import pytest

from social_nav_tools.pedestrian_model import Box
from social_nav_tools.worker_model import (
    CROSSING,
    WORKSTATION,
    Worker,
    WorkerConfig,
    _build_segments,
    _leg_state,
    _leg_time,
    active_segment,
    is_dwelling,
    plan_workers,
    pose_at_time,
)

# Keep-outs matching factory.world (racks, machines, charger, guard rail), map frame.
FACTORY_OBSTACLES = (
    Box(1.0, 1.5, 2.0, 3.2),
    Box(2.6, 0.7, 4.0, 3.2),
    Box(5.2, 1.0, 9.3, 3.2),
    Box(0.8, -3.2, 3.0, -1.5),
    Box(5.2, -3.2, 9.3, -1.0),
    Box(9.5, -2.9, 10.7, -1.5),
    Box(9.5, 1.6, 10.7, 3.2),
    Box(-3.85, -5.6, -3.35, -4.8),
    Box(-3.8, 3.29, -1.4, 3.41),
)

WORKSTATIONS = (
    (-3.2, 2.4), (-1.4, 2.6), (-3.2, -0.4), (-1.6, -2.4),
    (-3.2, -2.6), (0.4, 1.9), (0.4, -1.8),
)

CROSSING_ROUTES = (((4.6, 4.2), (4.6, -4.2)),)


def make_cfg(**overrides):
    base = dict(
        num_workstation=2,
        num_crossing=1,
        seed=42,
        workstations=WORKSTATIONS,
        crossing_routes=CROSSING_ROUTES,
        obstacles=FACTORY_OBSTACLES,
        robot_start=(0.0, 0.0),
        robot_keepout=1.5,
        walkable=(-3.8, 10.8, -6.3, 6.3),
        min_separation=1.2,
        speed=0.9,
        accel=0.6,
        dwell_time=6.0,
        dwell_jitter=2.0,
        speed_jitter=0.15,
        stations_per_worker=3,
    )
    base.update(overrides)
    return WorkerConfig(**base)


# --- determinism -----------------------------------------------------------------------------

def test_seed_is_reproducible():
    a = plan_workers(make_cfg(seed=42))
    b = plan_workers(make_cfg(seed=42))
    assert a == b
    # Crossing worker is planned first, then the workstation workers.
    assert [w.role for w in a] == [CROSSING, WORKSTATION, WORKSTATION]
    assert [w.name for w in a] == ["worker_cross_0", "worker_ws_1", "worker_ws_2"]


def test_different_seed_changes_plan():
    a = plan_workers(make_cfg(seed=42))
    b = plan_workers(make_cfg(seed=7))
    # Roles/counts are the same, but station assignment and jitter differ.
    assert a != b


# --- spawn safety ----------------------------------------------------------------------------

def test_no_spawn_inside_obstacles():
    for seed in range(20):
        for w in plan_workers(make_cfg(seed=seed)):
            for box in FACTORY_OBSTACLES:
                assert not box.contains(*w.start), f"{w.name} (seed {seed}) spawned in {box}"


def test_spawns_respect_robot_keepout_and_separation():
    cfg = make_cfg()
    workers = plan_workers(cfg)
    for i, a in enumerate(workers):
        assert math.hypot(a.start[0], a.start[1]) >= cfg.robot_keepout - 1e-6
        for b in workers[i + 1:]:
            d = math.hypot(a.start[0] - b.start[0], a.start[1] - b.start[1])
            assert d >= cfg.min_separation - 1e-6


def test_spawns_inside_walkable():
    cfg = make_cfg()
    xmin, xmax, ymin, ymax = cfg.walkable
    for w in plan_workers(cfg):
        x, y = w.start
        assert xmin <= x <= xmax and ymin <= y <= ymax


def test_dwell_stations_are_clear_of_machinery():
    # Every station a workstation worker stops at must be outside the racks/machines, not just
    # the spawn, or the worker would appear to work inside a shelf.
    for w in plan_workers(make_cfg(seed=3)):
        if w.role != WORKSTATION:
            continue
        for station in w.route:
            for box in FACTORY_OBSTACLES:
                assert not box.contains(*station), f"{w.name} works inside {box}"


# --- crossing behaviour ----------------------------------------------------------------------

def test_crossing_worker_crosses_the_robot_lane():
    workers = plan_workers(make_cfg(num_workstation=0, num_crossing=1))
    assert len(workers) == 1 and workers[0].role == CROSSING
    ys = [wp[1] for wp in workers[0].route]
    # Route straddles the robot lane (spans from north of it to south of it, lane is +-1 m).
    assert max(ys) > 1.0 and min(ys) < -1.0


def test_multiple_crossers_are_staggered():
    workers = plan_workers(make_cfg(num_workstation=0, num_crossing=2))
    xs = [w.start[0] for w in workers]
    assert abs(xs[0] - xs[1]) >= make_cfg().min_separation - 1e-6


# --- dwell state machine ---------------------------------------------------------------------

def test_workstation_worker_dwells_and_moves():
    workers = plan_workers(make_cfg(num_workstation=1, num_crossing=0))
    w = workers[0]
    # Sample the whole cycle: there must be both a stopped phase and a moving phase.
    dwell_seen = moving_seen = False
    for k in range(400):
        t = w.period * k / 400.0
        _, _, _, vx, vy = pose_at_time(w, t)
        if is_dwelling(w, t):
            assert math.hypot(vx, vy) < 1e-6
            dwell_seen = True
        elif math.hypot(vx, vy) > 1e-6:
            moving_seen = True
    assert dwell_seen and moving_seen


def test_dwell_pose_sits_exactly_on_a_station():
    w = plan_workers(make_cfg(num_workstation=1, num_crossing=0))[0]
    for seg in w.segments:
        if not seg.dwell:
            continue
        t = 0.5 * (seg.t0 + seg.t1)
        x, y, _, vx, vy = pose_at_time(w, t)
        assert (x, y) == pytest.approx((seg.x0, seg.y0), abs=1e-9)
        assert (vx, vy) == (0.0, 0.0)
        assert (seg.x0, seg.y0) in [tuple(p) for p in w.route]


def test_timeline_is_cyclic():
    w = plan_workers(make_cfg(num_workstation=1, num_crossing=0))[0]
    a = pose_at_time(w, 0.0)
    b = pose_at_time(w, w.period)          # exactly one cycle later
    c = pose_at_time(w, 3.0 * w.period)    # three cycles later
    assert a == pytest.approx(b, abs=1e-6)
    assert a == pytest.approx(c, abs=1e-6)


# --- trapezoidal kinematics ------------------------------------------------------------------

def test_leg_reaches_end_and_starts_stops_at_rest():
    length, v, a = 5.0, 0.9, 0.6
    total = _leg_time(length, v, a)
    s0, spd0 = _leg_state(length, v, a, 0.0)
    s1, spd1 = _leg_state(length, v, a, total)
    assert s0 == pytest.approx(0.0, abs=1e-9)
    assert spd0 == pytest.approx(0.0, abs=1e-9)
    assert s1 == pytest.approx(length, abs=1e-6)
    assert spd1 == pytest.approx(0.0, abs=1e-6)


def test_leg_speed_never_exceeds_cruise():
    length, v, a = 6.0, 0.9, 0.6
    total = _leg_time(length, v, a)
    for k in range(200):
        _, spd = _leg_state(length, v, a, total * k / 200.0)
        assert spd <= v + 1e-9


def test_short_leg_uses_triangular_profile():
    # Too short to reach cruise: peak speed is sqrt(a*L) < v, and it still ends at rest.
    length, v, a = 0.2, 0.9, 0.6
    total = _leg_time(length, v, a)
    peak = 0.0
    for k in range(200):
        _, spd = _leg_state(length, v, a, total * k / 200.0)
        peak = max(peak, spd)
    assert peak < v
    assert peak == pytest.approx(math.sqrt(a * length), abs=1e-2)


def test_travel_speed_matches_direction_and_magnitude():
    w = Worker(id=0, name="w", role=WORKSTATION, route=[(0.0, 0.0), (3.0, 4.0)],
               speed=1.0, accel=0.5, dwell_time=0.0)
    segs, period = _build_segments(w.route, w.speed, w.accel, 0.0)
    w.segments, w.period = segs, period
    travel = next(s for s in segs if not s.dwell)
    t = 0.5 * (travel.t0 + travel.t1)  # mid-leg, cruising
    _, _, yaw, vx, vy = pose_at_time(w, t)
    assert math.hypot(vx, vy) == pytest.approx(1.0, abs=1e-6)
    assert yaw == pytest.approx(math.atan2(4.0, 3.0), abs=1e-6)


# --- error handling --------------------------------------------------------------------------

def test_empty_workstation_pool_raises():
    with pytest.raises(ValueError):
        plan_workers(make_cfg(num_workstation=1, num_crossing=0, workstations=()))


def test_empty_crossing_routes_raises():
    with pytest.raises(ValueError):
        plan_workers(make_cfg(num_workstation=0, num_crossing=1, crossing_routes=()))


def test_single_point_route_is_stationary_dwell():
    segs, period = _build_segments([(2.0, 2.0)], speed=0.9, accel=0.6, dwell_time=5.0)
    w = Worker(id=0, name="w", role=WORKSTATION, route=[(2.0, 2.0)],
               speed=0.9, accel=0.6, dwell_time=5.0, segments=segs, period=period)
    x, y, _, vx, vy = pose_at_time(w, 3.3)
    assert (x, y, vx, vy) == (2.0, 2.0, 0.0, 0.0)
    assert is_dwelling(w, 3.3)
