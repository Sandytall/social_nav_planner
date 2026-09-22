"""Unit tests for the deterministic pedestrian planning logic (no ROS/Gazebo needed)."""
import math

import pytest

from social_nav_tools.pedestrian_model import (
    CROSSING,
    Box,
    Pedestrian,
    SpawnConfig,
    plan_pedestrians,
    pose_at_time,
)

# Keep-outs matching urban.world (building facades + street furniture), map frame.
URBAN_OBSTACLES = (
    Box(-1.0, 1.6, 3.5, 4.6),      # building A
    Box(7.5, 1.6, 13.0, 4.6),      # building B
    Box(2.85, -1.35, 3.15, -1.05),  # lamp post
    Box(6.85, -1.35, 7.15, -1.05),  # bin
    Box(9.4, 0.9, 10.6, 1.3),      # bench
)


def make_cfg(**overrides):
    base = dict(
        num_humans=3,
        seed=42,
        spawn_region=(1.0, 12.0, -0.5, 1.4),
        behavior_profile="mixed",
        min_separation=1.0,
        obstacles=URBAN_OBSTACLES,
        robot_start=(0.0, 0.0),
        robot_keepout=1.5,
        speed=0.9,
    )
    base.update(overrides)
    return SpawnConfig(**base)


def test_seed_is_reproducible():
    a = plan_pedestrians(make_cfg(seed=42))
    b = plan_pedestrians(make_cfg(seed=42))
    assert a == b
    # And the identities/behaviours are the expected shape.
    assert [p.name for p in a] == ["pedestrian_0", "pedestrian_1", "pedestrian_2"]


def test_different_seed_changes_walking_spawns():
    a = plan_pedestrians(make_cfg(seed=42))
    b = plan_pedestrians(make_cfg(seed=7))
    assert a != b


def test_no_spawn_inside_obstacles():
    people = plan_pedestrians(make_cfg())
    for ped in people:
        for box in URBAN_OBSTACLES:
            assert not box.contains(*ped.start), f"{ped.name} spawned in {box}"


def test_min_separation_and_robot_keepout():
    cfg = make_cfg()
    people = plan_pedestrians(cfg)
    for i, a in enumerate(people):
        assert math.hypot(a.start[0], a.start[1]) >= cfg.robot_keepout - 1e-6
        for b in people[i + 1:]:
            d = math.hypot(a.start[0] - b.start[0], a.start[1] - b.start[1])
            assert d >= cfg.min_separation - 1e-6


def test_crossing_profile_crosses_the_lane():
    people = plan_pedestrians(make_cfg(behavior_profile="crossing"))
    assert all(p.behavior == CROSSING for p in people)
    for ped in people:
        ys = [wp[1] for wp in ped.waypoints]
        # Route straddles the sidewalk lane (spans from north of it to south of it).
        assert max(ys) > 1.4 and min(ys) < -1.4


def test_mixed_profile_has_a_crosser():
    people = plan_pedestrians(make_cfg(behavior_profile="mixed"))
    assert any(p.behavior == CROSSING for p in people)


def test_unknown_profile_raises():
    with pytest.raises(ValueError):
        plan_pedestrians(make_cfg(behavior_profile="teleport"))


def test_infeasible_region_raises():
    # A tiny region fully inside building A leaves no room for a walking spawn.
    with pytest.raises(ValueError):
        plan_pedestrians(make_cfg(
            behavior_profile="walking", spawn_region=(0.0, 1.0, 2.0, 3.0)))


def test_pose_at_time_starts_at_route_start_and_returns():
    ped = Pedestrian(id=0, name="p", behavior="walking",
                     waypoints=[(1.0, 0.5), (5.0, 0.5)], speed=1.0)
    x0, y0, _, _, _ = pose_at_time(ped, 0.0)
    assert (round(x0, 6), round(y0, 6)) == (1.0, 0.5)

    seg = 4.0
    period = 2.0 * seg / ped.speed
    xh, _, _, _, _ = pose_at_time(ped, period / 2.0)  # far end
    assert xh == pytest.approx(5.0, abs=1e-6)
    xr, yr, _, _, _ = pose_at_time(ped, period)  # back to start
    assert (xr, yr) == pytest.approx((1.0, 0.5), abs=1e-6)


def test_pose_at_time_speed_matches():
    ped = Pedestrian(id=0, name="p", behavior="walking",
                     waypoints=[(0.0, 0.0), (3.0, 4.0)], speed=1.3)
    _, _, _, vx, vy = pose_at_time(ped, 0.5)
    assert math.hypot(vx, vy) == pytest.approx(1.3, abs=1e-6)


def test_stationary_route_is_safe():
    ped = Pedestrian(id=0, name="p", behavior="walking",
                     waypoints=[(2.0, 2.0), (2.0, 2.0)], speed=1.0)
    x, y, yaw, vx, vy = pose_at_time(ped, 3.3)
    assert (x, y, vx, vy) == (2.0, 2.0, 0.0, 0.0)
    assert yaw == 0.0
