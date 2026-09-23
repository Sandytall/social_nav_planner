"""Pure converters from ROS messages to the RL core's plain dataclasses.

Kept free of rclpy so they unit-test with duck-typed fake messages: the GazeboBackend uses
them to turn live /odom, /social_nav/humans and the classical prediction into the exact same
RobotState / Human / Prediction the MockBackend produces, so the observation is identical
whether training runs in the accelerated env or in Gazebo.
"""
import math

from social_nav_rl.observation import Human, Prediction


def quat_to_yaw(q) -> float:
    """Yaw (rad) from a geometry_msgs/Quaternion-like object."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def odom_to_robot(msg, goal) -> dict:
    """nav_msgs/Odometry -> the backend robot dict (goal is (gx, gy))."""
    p = msg.pose.pose.position
    tw = msg.twist.twist
    return {"x": p.x, "y": p.y, "yaw": quat_to_yaw(msg.pose.pose.orientation),
            "v": math.hypot(tw.linear.x, tw.linear.y), "w": tw.angular.z,
            "goal_x": goal[0], "goal_y": goal[1]}


def humans_from_array(msg):
    """social_nav_msgs/HumanArray -> [Human]."""
    out = []
    for h in msg.humans:
        p = h.pose.position
        v = h.velocity.linear
        out.append(Human(id=int(h.id), x=p.x, y=p.y, vx=v.x, vy=v.y,
                         yaw=quat_to_yaw(h.pose.orientation),
                         group_id=int(getattr(h, "group_id", -1))))
    return out


def predictions_from_array(msg):
    """social_nav_msgs/HumanPredictionArray -> {human_id: Prediction}."""
    preds = {}
    for pt in msg.predictions:
        poses = [(ps.position.x, ps.position.y) for ps in pt.predicted_poses]
        preds[int(pt.human_id)] = Prediction(human_id=int(pt.human_id), poses=poses,
                                             confidence=list(pt.prediction_confidence))
    return preds
