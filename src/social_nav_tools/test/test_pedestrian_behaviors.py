import math

from social_nav_tools.pedestrian_model import (
    HEAD_ON, TURNING, Pedestrian, SpawnConfig, plan_pedestrians, pose_at_time)


def _cfg(profile, num_humans=1, seed=42):
    return SpawnConfig(num_humans=num_humans, seed=seed,
                       spawn_region=(1.0, 12.0, -0.5, 1.4), behavior_profile=profile)


def test_head_on_walks_toward_robot():
    p = plan_pedestrians(_cfg(HEAD_ON))[0]
    assert p.behavior == HEAD_ON
    assert abs(p.waypoints[0][1]) < 0.6 and abs(p.waypoints[-1][1]) < 0.6  # on the robot lane
    assert p.waypoints[0][0] > p.waypoints[-1][0]  # starts east, walks toward the origin


def test_turning_route_has_a_bend():
    p = plan_pedestrians(_cfg(TURNING))[0]
    assert p.behavior == TURNING
    assert len(p.waypoints) >= 3
    (x0, y0), (x1, y1), (x2, y2) = p.waypoints[0], p.waypoints[1], p.waypoints[2]
    d1 = (x1 - x0, y1 - y0)
    d2 = (x2 - x1, y2 - y1)
    dot = d1[0] * d2[0] + d1[1] * d2[1]
    assert abs(dot) < 0.99 * math.hypot(*d1) * math.hypot(*d2)  # genuinely changes direction


def test_polyline_pose_traverses_each_segment():
    ped = Pedestrian(id=0, name="p", behavior=TURNING,
                     waypoints=[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0)], speed=1.0)
    # total length 8, out-and-back period 16; arc 2 is on segment 1, arc 6 on segment 2.
    x, y, _, _, _ = pose_at_time(ped, 2.0)
    assert abs(x - 2.0) < 1e-6 and abs(y - 0.0) < 1e-6
    x, y, _, _, _ = pose_at_time(ped, 6.0)
    assert abs(x - 4.0) < 1e-6 and abs(y - 2.0) < 1e-6


def test_polyline_two_point_matches_straight_line():
    ped = Pedestrian(id=0, name="p", behavior="walking",
                     waypoints=[(0.0, 0.0), (6.0, 0.0)], speed=1.0)
    x, y, _, vx, _ = pose_at_time(ped, 3.0)  # 3 m along -> x = 3
    assert abs(x - 3.0) < 1e-6 and abs(y) < 1e-6 and vx > 0
