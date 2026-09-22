"""Ingest a rosbag2 recording into the same ``record`` dict the live runner produces.

This lets ``social-nav-analyze`` run the full offline layer (metrics, events, trajectory,
stats, failures, HTML report) on a real-robot or previously-recorded bag, not only on a
fresh Gazebo run. The bag must contain ``/odom`` (nav_msgs/Odometry), ``/social_nav/humans``
(social_nav_msgs/HumanArray) and, when available, ``/social_nav/debug/metrics``
(social_nav_msgs/PlannerMetrics).

The message->record conversion is a set of pure functions that take duck-typed message
objects, so they unit-test with tiny fakes and need neither ROS nor a real bag. Only
``load_bag`` touches rosbag2_py / rclpy, and it imports them lazily.
"""

import math

# Topic names this loader understands.
ODOM_TOPIC = "/odom"
HUMANS_TOPIC = "/social_nav/humans"
METRICS_TOPIC = "/social_nav/debug/metrics"


# --------------------------------------------------------------------------------------
# Pure message -> value conversions (unit-tested with fake msgs).
# --------------------------------------------------------------------------------------

def odom_xyv(msg):
    """(x, y, speed) from a nav_msgs/Odometry-like message."""
    p = msg.pose.pose.position
    lin = msg.twist.twist.linear
    return (p.x, p.y, math.hypot(lin.x, lin.y))


def humans_xy(msg):
    """[(x, y), ...] from a social_nav_msgs/HumanArray-like message."""
    return [(h.pose.position.x, h.pose.position.y) for h in msg.humans]


def metrics_dict(msg):
    """PlannerMetrics-like message -> the dict shape metrics.compute expects."""
    return {
        "p50_ms": float(msg.p50_ms),
        "p95_ms": float(msg.p95_ms),
        "p99_ms": float(msg.p99_ms),
        "max_ms": float(msg.max_ms),
        "deadline_misses": int(msg.deadline_misses),
        "actual_frequency_hz": float(msg.actual_frequency_hz),
    }


def _t0(*stamped_lists):
    """Earliest timestamp (ns) across every (stamp_ns, msg) list, or 0 if all empty."""
    stamps = [s for lst in stamped_lists for (s, _) in lst]
    return min(stamps) if stamps else 0


def build_record(scenario, run, odom_items, human_items, metric_items,
                 status="UNKNOWN", planner_name=None):
    """Assemble a ``record`` dict from stamped message lists.

    Each ``*_items`` is a list of ``(stamp_ns, msg)``. Timestamps are converted to seconds
    relative to the earliest message across all topics, matching the live runner's
    "seconds from run start" convention.
    """
    t0 = _t0(odom_items, human_items, metric_items)

    def rel(ns):
        return (ns - t0) / 1e9

    odom = [(round(rel(s), 6), *odom_xyv(m)) for (s, m) in odom_items]
    humans = [(round(rel(s), 6), humans_xy(m)) for (s, m) in human_items]

    series = []
    for (s, m) in metric_items:
        d = metrics_dict(m)
        series.append((round(rel(s), 6), d["p50_ms"], d["p95_ms"], d["p99_ms"],
                       d["max_ms"], d["actual_frequency_hz"], d["deadline_misses"]))

    record = {
        "scenario": scenario,
        "run": run,
        "status": status,
        "odom": odom,
        "humans": humans,
        "planner": metrics_dict(metric_items[-1][1]) if metric_items else None,
        "source": "rosbag2",
    }
    if series:
        record["planner_series"] = series
    if planner_name is not None:
        record["planner_name"] = planner_name
    return record


# --------------------------------------------------------------------------------------
# rosbag2 reader (lazy ROS imports).
# --------------------------------------------------------------------------------------

def load_bag(bag_dir, scenario="bag", run=1, status="UNKNOWN", storage_id=""):
    """Read a rosbag2 directory and return a ``record`` dict.

    ``storage_id`` is auto-detected from metadata.yaml when left empty (sqlite3 or mcap).
    Only the three known topics are deserialised; anything else in the bag is skipped.
    """
    import os

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    if not storage_id:
        storage_id = _detect_storage_id(bag_dir)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    type_by_topic = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {ODOM_TOPIC, HUMANS_TOPIC, METRICS_TOPIC} & set(type_by_topic)
    msg_class = {name: get_message(type_by_topic[name]) for name in wanted}

    odom_items, human_items, metric_items = [], [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(data, msg_class[topic])
        if topic == ODOM_TOPIC:
            odom_items.append((stamp, msg))
        elif topic == HUMANS_TOPIC:
            human_items.append((stamp, msg))
        elif topic == METRICS_TOPIC:
            metric_items.append((stamp, msg))

    if not odom_items:
        raise ValueError(
            f"bag '{bag_dir}' has no {ODOM_TOPIC} messages; cannot build a record. "
            f"Topics present: {sorted(type_by_topic)}")

    _ = os  # keep import meaningful for future path handling
    return build_record(scenario, run, odom_items, human_items, metric_items,
                        status=status)


def _detect_storage_id(bag_dir):
    """Read metadata.yaml to pick the storage plugin; default sqlite3."""
    import os

    meta = os.path.join(bag_dir, "metadata.yaml")
    try:
        import yaml

        with open(meta) as f:
            data = yaml.safe_load(f)
        return data["rosbag2_bagfile_information"]["storage_identifier"]
    except Exception:  # noqa: BLE001 - fall back to the common default
        return "sqlite3"
