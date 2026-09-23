"""Extract 2D static-obstacle footprints from a Gazebo Classic .world (SDF).

The accelerated RL mock env must contain the SAME static geometry the robot's lidar sees in
Gazebo, otherwise a policy trained in the mock drives blindly into real racks/walls. This parses
a world's static box collisions into axis-aligned 2D rectangles (xmin, ymin, xmax, ymax), keeping
only boxes tall enough to cross the lidar plane so floor markings (lane paint, walkways, charging
pads: a centimetre tall) are excluded.

Only box geometries are handled (every obstacle in these worlds is a box); non-box collisions are
skipped. Rotation about z (yaw) is honoured via the footprint's axis-aligned bounding box; roll and
pitch are ignored (obstacles here stand upright).
"""
import math
import os
import xml.etree.ElementTree as ET
from functools import lru_cache

LIDAR_MIN_HEIGHT = 0.3   # a box must rise above this (m) to be seen by the ~0.4 m lidar


def _pose(el):
    """(x, y, z, yaw) from an element's <pose>, defaulting to zeros when absent."""
    p = el.find("pose")
    if p is None or not (p.text or "").strip():
        return (0.0, 0.0, 0.0, 0.0)
    v = [float(x) for x in p.text.split()]
    x = v[0] if len(v) > 0 else 0.0
    y = v[1] if len(v) > 1 else 0.0
    z = v[2] if len(v) > 2 else 0.0
    yaw = v[5] if len(v) >= 6 else 0.0
    return (x, y, z, yaw)


def _footprint(cx, cy, yaw, sx, sy):
    """Axis-aligned bbox of a box of planar size (sx, sy) centred at (cx, cy), rotated by yaw."""
    hx, hy = sx / 2.0, sy / 2.0
    c, s = math.cos(yaw), math.sin(yaw)
    xs, ys = [], []
    for lx, ly in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
        xs.append(cx + c * lx - s * ly)
        ys.append(cy + s * lx + c * ly)
    return (min(xs), min(ys), max(xs), max(ys))


def extract_obstacles(world_path, min_height=LIDAR_MIN_HEIGHT):
    """Static box collisions in `world_path` as [(xmin, ymin, xmax, ymax), ...] (lidar-height only)."""
    root = ET.parse(world_path).getroot()
    boxes = []
    for model in root.iter("model"):
        st = model.find("static")
        if st is None or (st.text or "").strip().lower() not in ("1", "true"):
            continue
        for link in model.iter("link"):
            lx, ly, lz, lyaw = _pose(link)
            for col in link.iter("collision"):
                size = col.find("./geometry/box/size")
                if size is None or not (size.text or "").strip():
                    continue
                sx, sy, sz = (float(v) for v in size.text.split())
                cx0, cy0, cz0, cyaw0 = _pose(col)
                # compose link pose with the collision's local pose (translation + yaw)
                c, s = math.cos(lyaw), math.sin(lyaw)
                cx = lx + c * cx0 - s * cy0
                cy = ly + s * cx0 + c * cy0
                cz = lz + cz0
                top = cz + sz / 2.0
                if sz < min_height or top < min_height:
                    continue                      # floor paint / thin markings: not an obstacle
                boxes.append(_footprint(cx, cy, lyaw + cyaw0, sx, sy))
    return boxes


@lru_cache(maxsize=16)
def obstacles_for_world(world_path):
    """Cached tuple of obstacle boxes for a world file (returns () if the file is missing)."""
    if not world_path or not os.path.isfile(world_path):
        return ()
    return tuple(extract_obstacles(world_path))


def resolve_world_path(world_filename):
    """Best-effort absolute path to worlds/<file>: installed share first, then the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("social_nav_sim"), "worlds", world_filename)
        if os.path.isfile(p):
            return p
    except Exception:  # noqa: BLE001 - ament not available (bare tests); fall through to source
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.join(here, "..", "..", "social_nav_sim", "worlds", world_filename)
    return cand if os.path.isfile(cand) else None


def obstacles_for_env(world_filename):
    """Obstacle boxes for an environment's world file name (e.g. 'factory.world'); () if unresolved."""
    return obstacles_for_world(resolve_world_path(world_filename))
