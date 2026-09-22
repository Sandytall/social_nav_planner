import math

from social_nav_tools.pedestrian_model import (
    BLOCKER, MERGE, SAME_DIRECTION, STOP_GO, Pedestrian, SpawnConfig,
    plan_pedestrians, pose_at_time)


def _cfg(profile, n=1, seed=42):
    return SpawnConfig(num_humans=n, seed=seed, spawn_region=(1.0, 12.0, -1.4, 1.4),
                       behavior_profile=profile)


def test_same_direction_heads_east_on_lane():
    p = plan_pedestrians(_cfg(SAME_DIRECTION))[0]
    assert p.behavior == SAME_DIRECTION
    assert p.waypoints[-1][0] > p.waypoints[0][0]   # travels east (robot overtakes)
    assert abs(p.waypoints[0][1]) < 0.6             # on the robot's lane


def test_blocker_is_stationary_on_the_path():
    p = plan_pedestrians(_cfg(BLOCKER))[0]
    for t in (0.0, 2.0, 5.0):
        _, _, _, vx, vy = pose_at_time(p, t)
        assert math.hypot(vx, vy) == 0.0
    assert pose_at_time(p, 0.0)[:2] == pose_at_time(p, 5.0)[:2]
    assert abs(p.start[1]) < 1e-9                    # sits on y=0, the robot's lane


def test_stop_go_pauses_then_resumes():
    p = plan_pedestrians(_cfg(STOP_GO))[0]
    assert p.stop_start >= 0.0 and p.stop_duration > 0.0
    moving_before = pose_at_time(p, p.stop_start - 0.5)
    assert math.hypot(moving_before[3], moving_before[4]) > 0.0
    frozen = pose_at_time(p, p.stop_start + 1.0)
    assert frozen[3] == 0.0 and frozen[4] == 0.0
    assert pose_at_time(p, p.stop_start + 2.0)[:2] == pose_at_time(p, p.stop_start)[:2]
    moving_after = pose_at_time(p, p.stop_start + p.stop_duration + 0.5)
    assert math.hypot(moving_after[3], moving_after[4]) > 0.0


def test_merge_two_groups_converge_to_centreline():
    people = plan_pedestrians(_cfg(MERGE, n=4))
    assert len(people) == 4
    assert sorted({p.group_id for p in people}) == [0, 1]
    for p in people:
        assert len(p.waypoints) >= 3                 # a bend toward the centreline
        assert abs(p.waypoints[-1][1]) < 0.2         # ends on the shared centreline
    starts = {p.group_id: p.waypoints[0][1] for p in people}
    assert starts[0] * starts[1] < 0                 # groups start on opposite lanes


def test_high_density_mixed_places_all():
    people = plan_pedestrians(SpawnConfig(
        num_humans=12, seed=42, spawn_region=(1.0, 12.0, -1.4, 1.4),
        behavior_profile="mixed"))
    assert len(people) == 12


def test_two_point_route_still_moves():
    ped = Pedestrian(id=0, name="p", behavior="walking",
                     waypoints=[(0.0, 0.0), (6.0, 0.0)], speed=1.0)
    x, y, _, vx, _ = pose_at_time(ped, 3.0)
    assert abs(x - 3.0) < 1e-6 and abs(y) < 1e-6 and vx > 0
