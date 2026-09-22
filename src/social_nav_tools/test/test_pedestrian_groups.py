import math

from social_nav_tools.pedestrian_model import (
    GROUP, SpawnConfig, _group_sizes, plan_pedestrians, pose_at_time)


def _cfg(num_humans=4, seed=42, group_size=4):
    return SpawnConfig(
        num_humans=num_humans, seed=seed,
        spawn_region=(1.0, 12.0, -0.5, 1.4),
        behavior_profile=GROUP, group_size=group_size)


def test_group_sizes_split():
    assert _group_sizes(7, 3) == [3, 4]   # trailing 1 merged into the previous group
    assert _group_sizes(6, 3) == [3, 3]
    assert _group_sizes(4, 3) == [4]
    assert _group_sizes(2, 3) == [2]


def test_group_size_clamped_to_five():
    assert _group_sizes(10, 99) == [5, 5]


def test_one_group_all_same_id():
    people = plan_pedestrians(_cfg(num_humans=4, group_size=4))
    assert len(people) == 4
    assert {p.group_id for p in people} == {0}
    assert all(p.behavior == GROUP for p in people)


def test_multiple_groups_have_distinct_ids():
    people = plan_pedestrians(_cfg(num_humans=6, group_size=3))
    assert len(people) == 6
    assert sorted({p.group_id for p in people}) == [0, 1]


def test_members_spread_by_group_spacing():
    people = plan_pedestrians(_cfg(num_humans=4, group_size=4))
    ys = sorted(p.start[1] for p in people)
    gaps = [ys[i + 1] - ys[i] for i in range(len(ys) - 1)]
    assert all(abs(g - 0.7) < 1e-6 for g in gaps)


def test_group_plan_is_deterministic():
    a = plan_pedestrians(_cfg(seed=7))
    b = plan_pedestrians(_cfg(seed=7))
    assert [(p.name, p.waypoints, p.group_id) for p in a] == \
           [(p.name, p.waypoints, p.group_id) for p in b]


def test_group_member_moves():
    p = plan_pedestrians(_cfg(num_humans=2, group_size=2))[0]
    _, _, _, vx, vy = pose_at_time(p, 1.0)
    assert math.hypot(vx, vy) > 0.0
