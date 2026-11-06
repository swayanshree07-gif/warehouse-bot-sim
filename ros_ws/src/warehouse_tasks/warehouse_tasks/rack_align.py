"""Lidar rack-face measurement for precise docking (Day 20). Pure Python.

AMCL alone leaves 2-7 cm of error at the dock pose (Day 20 baseline: 2/5
trials within 3 cm). The rack itself is the best reference: from the Nav2
approach pose, facing the rack, the 2D scan sees the rack's front face as one
straight segment that ends at both rack corners. `measure_face` fits that line
(distance + angle) and finds the two corners; with the slot's known offset from
the rack centre that gives the dock pose in the robot's own base frame, without
using the map. The caller turns it into an odom-frame target (odom is smooth
over the last metre) and line-tracks to it.

Frames: base frame = base_link, x forward, y left. The face normal n points from
the robot into the rack; t = n rotated +90 deg (the robot's left along the face).

Corner accuracy is limited by the beam spacing on the face (1 deg beams), so
each corner is placed half-way between the last beam that hits the face and the
next beam's intersection with the face line, and the two corners are weighted
by 1/spacing^2. Expected error at 0.5 m laser->face: ~0.5 cm for a corner
0.25 m to the side, ~1.4 cm at 0.75 m.
"""
import math


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _fit(pts):
    """Least squares x = a + b*y (the face is roughly perpendicular to x)."""
    n = len(pts)
    my = sum(p[1] for p in pts) / n
    mx = sum(p[0] for p in pts) / n
    syy = sum((p[1] - my) ** 2 for p in pts)
    if syy < 1e-9:
        return None
    b = sum((p[1] - my) * (p[0] - mx) for p in pts) / syy
    return mx - b * my, b


def measure_face(ranges, angle_min, angle_inc, range_min, range_max, laser=(0.2, 0.0, 0.0),
                 expect_dist=0.7, window=0.3, face_len=1.5, len_tol=0.05, inlier_tol=0.02):
    """Find the rack face ahead of the robot.

    ranges/angle_* describe a planar scan in the laser frame, whose pose in the
    base frame is laser = (x, y, yaw). expect_dist = expected base_link -> face
    distance. Returns (result, reason): result is a dict with
      d        distance base_link -> face line along n (m)
      n_yaw    angle of the face normal n in the base frame (rad)
      s_lo/s_hi corner positions along t (m, base_link projects to s = 0)
      s_mid    face centre along t; spacing_lo/hi corner uncertainty (beam spacing)
      n_pts    inliers
    or None and the reason it failed."""
    lx, ly, lyaw = laser
    beams = []
    for i, r in enumerate(ranges):
        if not (range_min < r < range_max) or math.isnan(r):
            continue
        a = lyaw + angle_min + i * angle_inc
        x, y = lx + r * math.cos(a), ly + r * math.sin(a)
        if abs(x - expect_dist) < window and abs(y) < face_len + 1.0:
            beams.append((i, x, y))
    if len(beams) < 6:
        return None, f"only {len(beams)} scan points near the rack face"
    # seed the fit with the beams straight ahead (they always hit the face when
    # the robot roughly faces the rack), then grow it with inliers from all beams
    pts = [(x, y) for _, x, y in beams if abs(y) < 0.35]
    if len(pts) < 6:
        return None, f"only {len(pts)} scan points straight ahead"
    keep = beams
    for _ in range(4):
        fit = _fit(pts)
        if fit is None:
            return None, "degenerate face fit"
        a, b = fit
        norm = math.sqrt(1 + b * b)
        keep = [(i, x, y) for i, x, y in beams if abs(x - a - b * y) / norm < inlier_tol]
        if len(keep) < 6:
            return None, f"only {len(keep)} face inliers"
        pts = [(x, y) for _, x, y in keep]
    a, b = _fit(pts)
    norm = math.sqrt(1 + b * b)
    nx, ny = 1 / norm, -b / norm          # into the rack
    tx, ty = -ny, nx                      # robot's left along the face
    d = a / norm
    # largest contiguous run along the face (gap < 0.2 m)
    proj = sorted(((x * tx + y * ty), i) for i, x, y in keep)
    runs, cur = [], [proj[0]]
    for p in proj[1:]:
        if p[0] - cur[-1][0] > 0.2:
            runs.append(cur)
            cur = []
        cur.append(p)
    runs.append(cur)
    run = max(runs, key=len)
    inl = {i for _, i in run}

    def beam_on_face(i):
        """(s, r) of beam i's intersection with the face line, or None."""
        ang = lyaw + angle_min + i * angle_inc
        ux, uy = math.cos(ang), math.sin(ang)
        un = ux * nx + uy * ny
        if un < 1e-3:
            return None
        rr = (d - (lx * nx + ly * ny)) / un
        return (lx + rr * ux) * tx + (ly + rr * uy) * ty, rr

    def corner(s_end, i_end, outward):
        """Corner = half-way to the next beam outward, which must be a valid
        return that passes the face line (else the corner may be hidden)."""
        for j in (i_end - 1, i_end + 1):
            j %= len(ranges)
            if j in inl:
                continue
            hit = beam_on_face(j)
            if hit is None or (hit[0] - s_end) * outward <= 0:
                continue
            r = ranges[j]
            if not (range_min < r < range_max) or math.isnan(r) or r < hit[1] + 0.05:
                return None
            return 0.5 * (s_end + hit[0]), abs(hit[0] - s_end)
        return None

    c_lo = corner(run[0][0], run[0][1], -1)
    c_hi = corner(run[-1][0], run[-1][1], +1)
    if c_lo is None or c_hi is None:
        return None, "rack corner not confirmed by the next beam (hidden or out of range)"
    (s_lo, sp_lo), (s_hi, sp_hi) = c_lo, c_hi
    length = s_hi - s_lo
    if abs(length - face_len) > len_tol + 0.5 * (sp_lo + sp_hi):
        return None, f"face length {length:.3f} m, want {face_len:.2f} (corner hidden?)"
    w_lo, w_hi = 1 / max(sp_lo, 1e-3) ** 2, 1 / max(sp_hi, 1e-3) ** 2
    s_mid = (w_lo * (s_lo + face_len / 2) + w_hi * (s_hi - face_len / 2)) / (w_lo + w_hi)
    return {"d": d, "n_yaw": math.atan2(ny, nx), "s_lo": s_lo, "s_hi": s_hi, "s_mid": s_mid,
            "spacing_lo": sp_lo, "spacing_hi": sp_hi, "n_pts": len(run)}, ""


def slot_geometry(shelf, slot, dock_yaw_deg):
    """From registry entries: (lateral offset of the slot centre from the rack
    centre along the robot's left t, rack depth along n, face length)."""
    yaw = math.radians(dock_yaw_deg)
    lx, ly = -math.sin(yaw), math.cos(yaw)      # robot's left in the map frame
    ox = slot["center"][0] - shelf["center"][0]
    oy = slot["center"][1] - shelf["center"][1]
    sx, sy = shelf["size"][0], shelf["size"][1]
    ryaw = math.radians(shelf.get("yaw_deg", 0.0))
    # rack extent along the normal / along the face (rack axes vs dock heading)
    along_x = abs(math.cos(yaw - ryaw)) > abs(math.sin(yaw - ryaw))
    depth, face_len = (sx, sy) if along_x else (sy, sx)
    return ox * lx + oy * ly, depth, face_len


def dock_in_base(meas, slot_lateral, depth, standoff):
    """Dock pose (x, y, yaw) in the base frame from a face measurement: the
    slot centre is depth/2 behind the face at s = s_mid + slot_lateral, and the
    dock pose is `standoff` in front of it, facing n. Also returns the robot's
    errors relative to that pose: (along = distance still to go, lateral =
    robot left of the slot line, yaw = robot heading minus dock heading)."""
    ny = meas["n_yaw"]
    nx_, ny_ = math.cos(ny), math.sin(ny)
    tx, ty = -ny_, nx_
    s = meas["s_mid"] + slot_lateral
    cn = meas["d"] + depth / 2 - standoff
    x, y = cn * nx_ + s * tx, cn * ny_ + s * ty
    return (x, y, ny), {"along": cn, "lateral": -s, "yaw": wrap(-ny)}


def compose(pose, rel):
    """pose (x, y, yaw) of the base in a world frame o rel (x, y, yaw) in the base frame."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * rel[0] - s * rel[1], pose[1] + s * rel[0] + c * rel[1], wrap(pose[2] + rel[2]))
