import math
from types import SimpleNamespace as NS

from social_nav_rl import ros_convert as RC


def _quat(x=0.0, y=0.0, z=0.0, w=1.0):
    return NS(x=x, y=y, z=z, w=w)


def test_quat_to_yaw():
    assert abs(RC.quat_to_yaw(_quat())) < 1e-9
    q = _quat(z=math.sin(math.pi / 4), w=math.cos(math.pi / 4))   # yaw = pi/2
    assert abs(RC.quat_to_yaw(q) - math.pi / 2) < 1e-6


def test_odom_to_robot():
    msg = NS(pose=NS(pose=NS(position=NS(x=5.0, y=6.0), orientation=_quat())),
             twist=NS(twist=NS(linear=NS(x=0.5, y=0.0), angular=NS(z=0.1))))
    r = RC.odom_to_robot(msg, (9.0, 0.0))
    assert r["x"] == 5.0 and r["y"] == 6.0 and abs(r["v"] - 0.5) < 1e-9 and r["w"] == 0.1
    assert r["goal_x"] == 9.0


def test_humans_from_array():
    h = NS(id=1, pose=NS(position=NS(x=1.0, y=2.0), orientation=_quat()),
           velocity=NS(linear=NS(x=0.3, y=0.0)), group_id=0)
    out = RC.humans_from_array(NS(humans=[h]))
    assert len(out) == 1 and out[0].id == 1 and out[0].x == 1.0 and out[0].vx == 0.3


def test_predictions_from_array():
    pt = NS(human_id=1, predicted_poses=[NS(position=NS(x=1.0, y=1.0)),
                                         NS(position=NS(x=1.2, y=1.0))],
            prediction_confidence=[0.9, 0.7])
    d = RC.predictions_from_array(NS(predictions=[pt]))
    assert 1 in d and d[1].poses == [(1.0, 1.0), (1.2, 1.0)] and d[1].confidence == [0.9, 0.7]
