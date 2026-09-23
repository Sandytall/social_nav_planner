import math

import numpy as np

from social_nav_rl import features as F


def test_wrap_angle():
    assert abs(F.wrap_angle(3 * math.pi) - math.pi) < 1e-6
    assert abs(abs(F.wrap_angle(-3 * math.pi)) - math.pi) < 1e-6   # ±pi both valid wraps
    assert abs(F.wrap_angle(math.pi / 2) - math.pi / 2) < 1e-6


def test_to_robot_frame_identity_and_rotation():
    assert F.to_robot_frame(1, 0, 0, 0, 0.0) == (1.0, 0.0)
    fx, fy = F.to_robot_frame(1, 0, 0, 0, math.pi / 2)  # robot faces +y; point ahead is (0,-1)
    assert abs(fx) < 1e-6 and abs(fy + 1.0) < 1e-6


def test_ttc_closing_diverging_parallel():
    contact = 0.75
    assert abs(F.time_to_collision((3, 0), (-1, 0), contact) - 2.25) < 1e-6   # head-on
    assert F.time_to_collision((3, 0), (1, 0), contact) >= F.BIG_TTC          # diverging
    assert F.time_to_collision((3, 0), (0, 0), contact) >= F.BIG_TTC          # not closing


def test_social_zone_front_costs_more_than_rear():
    assert F.social_zone_cost(0.0, 0.0) == 1.0                # on the human
    assert F.social_zone_cost(0.5, 0.0) > F.social_zone_cost(-0.5, 0.0)  # front > rear


def test_safe_array_replaces_nonfinite():
    out = F.safe_array([1.0, float("nan"), float("inf"), -float("inf")])
    assert np.all(np.isfinite(out)) and out[0] == 1.0
