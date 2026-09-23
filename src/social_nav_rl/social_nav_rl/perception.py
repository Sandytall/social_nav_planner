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


def front_beam_angles(n_beams, fov_deg):
    """Robot-relative angles of `n_beams` forward beams evenly spanning +/- fov/2 (0 = ahead).

    A higher-resolution forward "depth sector" on top of the coarse 360-deg lidar: it packs more
    beams into the front cone (where the robot is heading), so it resolves narrow gaps and
    approaching obstacles the 30-deg/beam lidar blurs. Same robot-frame convention as the lidar.
    """
    if n_beams <= 0:
        return np.zeros(0, dtype=np.float32)
    half = math.radians(fov_deg) / 2.0
    if n_beams == 1:
        return np.array([0.0], dtype=np.float32)
    step = (2.0 * half) / (n_beams - 1)
    return np.array([-half + i * step for i in range(n_beams)], dtype=np.float32)


def raycast_front(x, y, yaw, boxes, n_beams=12, fov_deg=120.0, max_range=5.0):
    """Forward depth sector (raw ranges, m) by ray-casting `boxes` over the front cone (mock)."""
    out = np.full(n_beams, max_range, dtype=np.float32)
    for i, off in enumerate(front_beam_angles(n_beams, fov_deg)):
        ang = yaw + float(off)
        dx, dy = math.cos(ang), math.sin(ang)
        best = max_range
        for (xmin, ymin, xmax, ymax) in boxes:
            t = _ray_aabb(x, y, dx, dy, xmin, ymin, xmax, ymax)
            if t is not None and t < best:
                best = t
        out[i] = best
    return out


def downsample_scan_front(ranges, angle_min, angle_increment, n_beams=12, fov_deg=120.0,
                          max_range=5.0):
    """Bin a real /scan into the same forward-sector beams (min range per bin), for Gazebo."""
    out = np.full(n_beams, max_range, dtype=np.float32)
    half = math.radians(fov_deg) / 2.0
    step = (2.0 * half) / max(1, n_beams)
    for j in range(len(ranges)):
        r = ranges[j]
        if not math.isfinite(r) or r <= 0.0:
            continue
        a = angle_min + j * angle_increment
        a = (a + math.pi) % (2.0 * math.pi) - math.pi          # wrap to [-pi, pi)
        if a < -half or a >= half:
            continue                                           # outside the forward cone
        b = min(max(int((a + half) / step), 0), n_beams - 1)
        r = min(r, max_range)
        if r < out[b]:
            out[b] = r
    return out


def normalize_lidar(ranges, max_range):
    """Ranges (m) -> [0, 1], 1 = clear (>= max_range), 0 = touching."""
    return np.clip(np.asarray(ranges, dtype=np.float32) / max(max_range, 1e-6), 0.0, 1.0)


def corridor_features(ranges, max_range, n_beams):
    """Derived corridor geometry the raw beams make hard for an MLP to read: normalized front /
    front-left / front-right / left / right clearance, and the lateral centre-offset
    e = (left - right) / (left + right) in [-1, 1] (which way there is more room). Directly targets
    the 'person on one side + rack on the other -> no room that way' failure."""
    ranges = np.asarray(ranges, dtype=np.float32)
    ang = beam_angles(n_beams)
    inv = 1.0 / max(max_range, 1e-6)

    def sector_min(lo, hi):
        m = (ang >= lo) & (ang < hi)
        return float(ranges[m].min()) if bool(m.any()) else float(max_range)

    front = sector_min(-math.pi / 6, math.pi / 6)
    fl = sector_min(math.pi / 6, math.pi / 2)
    fr = sector_min(-math.pi / 2, -math.pi / 6)
    left = sector_min(math.pi / 3, 2 * math.pi / 3)
    right = sector_min(-2 * math.pi / 3, -math.pi / 3)
    e_center = (left - right) / (left + right + 1e-6)
    return np.array([front * inv, fl * inv, fr * inv, left * inv, right * inv,
                     float(np.clip(e_center, -1.0, 1.0))], dtype=np.float32)
