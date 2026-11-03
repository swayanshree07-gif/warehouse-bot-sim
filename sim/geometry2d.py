"""2D ground-truth geometry of the warehouse for sensor/map checks (pure Python).

Obstacles are axis-aligned rectangles (xmin, ymin, xmax, ymax) taken from
warehouse_layout: walls, plus racks and boxes that reach the given height.
`raycast` gives the exact distance a planar lidar should report.
"""
import math

import warehouse_layout as L


def _rect(cx, cy, sx, sy):
    return (cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)


def obstacles_at_height(z):
    """Rectangles of everything that intersects the horizontal plane at z."""
    rects = []
    for _, (cx, cy, cz), (sx, sy, sz) in L.walls():
        if cz - sz / 2 <= z <= cz + sz / 2:
            rects.append(_rect(cx, cy, sx, sy))
    sx, sy, sz = L.SHELF_SIZE
    for s in L.SHELVES:
        if z <= sz:
            rects.append(_rect(s.x, s.y, sx, sy))
    b = L.BOX_SIZE
    for box in L.BOXES:
        if box.z - b / 2 <= z <= box.z + b / 2:
            rects.append(_rect(box.x, box.y, b, b))
    return rects


def ray_rect(ox, oy, dx, dy, r):
    """Distance along unit ray (o, d) to rectangle r, or None (slab method)."""
    tmin, tmax = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, r[0], r[2]), (oy, dy, r[1], r[3])):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    if tmax < max(tmin, 0.0):
        return None
    return tmin if tmin >= 0 else 0.0


def raycast(ox, oy, angle, rects, max_range=math.inf):
    dx, dy = math.cos(angle), math.sin(angle)
    best = max_range
    for r in rects:
        t = ray_rect(ox, oy, dx, dy, r)
        if t is not None and t < best:
            best = t
    return best


def expected_scan(x, y, yaw, angles_deg, z, max_range):
    """Ranges a perfect planar lidar at (x, y, yaw rad), height z would see."""
    rects = obstacles_at_height(z)
    return [raycast(x, y, yaw + math.radians(a), rects, max_range) for a in angles_deg]
