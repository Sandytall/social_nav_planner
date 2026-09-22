"""Static-obstacle perception shared by both backends, in ONE robot-frame representation.

The policy gets a fixed-size down-sampled lidar: ``n_beams`` evenly spaced beams over the full
circle, each the nearest static-obstacle range in that direction, clipped to ``max_range`` and
normalised to [0, 1] (1 = clear). It is computed by ray-casting the mock's obstacle boxes and by
down-sampling Gazebo's real ``/scan`` into the SAME beams, so an obstacle-aware policy trained in
the mock sees the same input shape/geometry in Gazebo.

Beam i covers robot-relative angle ``-pi + (i + 0.5) * 2*pi/n_beams`` (beam 0 = behind, the middle
beam = straight ahead), so the vector is orientation-invariant.
"""
import math

import numpy as np


def beam_angles(n_beams):
    """Robot-relative angle of each beam centre, in [-pi, pi)."""
    step = 2.0 * math.pi / n_beams
    return np.array([-math.pi + (i + 0.5) * step for i in range(n_beams)], dtype=np.float32)


def _ray_aabb(px, py, dx, dy, xmin, ymin, xmax, ymax):
    """Nearest positive hit distance of ray (px,py)+t(dx,dy) with an AABB, or None (slab method)."""
    tmin, tmax = 0.0, math.inf
    for p, d, lo, hi in ((px, dx, xmin, xmax), (py, dy, ymin, ymax)):
        if abs(d) < 1e-12:
            if p < lo or p > hi:
                return None                     # parallel and outside the slab
        else:
            t1, t2 = (lo - p) / d, (hi - p) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return None
    return tmin if tmax >= 0.0 else None


def raycast_lidar(x, y, yaw, boxes, n_beams=12, max_range=5.0):
    """Down-sampled lidar ranges (m) by ray-casting `boxes` from pose (x, y, yaw)."""
    out = np.full(n_beams, max_range, dtype=np.float32)
    for i, off in enumerate(beam_angles(n_beams)):
        ang = yaw + float(off)
        dx, dy = math.cos(ang), math.sin(ang)
        best = max_range
        for (xmin, ymin, xmax, ymax) in boxes:
            t = _ray_aabb(x, y, dx, dy, xmin, ymin, xmax, ymax)
            if t is not None and t < best:
                best = t
        out[i] = best
    return out


def downsample_scan(ranges, angle_min, angle_increment, n_beams=12, max_range=5.0):
    """Bin a real /scan into the same `n_beams` robot-relative beams (min range per bin)."""
    out = np.full(n_beams, max_range, dtype=np.float32)
    step = 2.0 * math.pi / n_beams
    n = len(ranges)
    for j in range(n):
        r = ranges[j]
        if not math.isfinite(r) or r <= 0.0:
            continue
        a = angle_min + j * angle_increment
        a = (a + math.pi) % (2.0 * math.pi) - math.pi          # wrap to [-pi, pi)
        b = int((a + math.pi) / step)
        if b < 0 or b >= n_beams:
            b = min(max(b, 0), n_beams - 1)
        r = min(r, max_range)
        if r < out[b]:
            out[b] = r
    return out


def normalize_lidar(ranges, max_range):
    """Ranges (m) -> [0, 1], 1 = clear (>= max_range), 0 = touching."""
    return np.clip(np.asarray(ranges, dtype=np.float32) / max(max_range, 1e-6), 0.0, 1.0)
