"""Line-tracking control law for docking/undocking (Day 20). Pure Python.

The dock and approach poses of a slot lie on one line (the slot's access
line, direction = dock yaw). Docking drives forward along it until the
along-track distance to the dock point is used up; undocking reverses along
it to the approach point. The heading reference corrects the cross-track error
(lateral offset), so the robot converges onto the line instead of chasing a
point; that cannot stall near the goal (an earlier point-chasing version did when
AMCL jitter swung the bearing to the point by > 60 deg at d < 5 cm).
"""
import math

K_LAT = 3.0             # rad of heading correction per m of lateral error (via atan)
MAX_CORR = math.radians(25)
K_YAW = 1.5
W_MAX = 0.5
V_MIN = 0.03


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def errors(pose, target, line_yaw):
    """(remaining, lateral) of pose (x, y, yaw) w.r.t. target (x, y) on a line
    with direction line_yaw. remaining > 0: target is still ahead along +u;
    lateral > 0: robot is left of the line."""
    ux, uy = math.cos(line_yaw), math.sin(line_yaw)
    dx, dy = pose[0] - target[0], pose[1] - target[1]
    return -(dx * ux + dy * uy), -dx * uy + dy * ux


def track_cmd(pose, target, line_yaw, reverse=False, v_max=0.12, tol=0.005):
    """One control step. Returns (v, w, done, remaining, lateral).
    Forward: drive along +u (facing line_yaw) to target. Reverse: the robot
    faces line_yaw and backs along -u to target."""
    remaining, lat = errors(pose, target, line_yaw)
    togo = -remaining if reverse else remaining
    if togo <= tol:
        return 0.0, 0.0, True, remaining, lat
    corr = max(-MAX_CORR, min(MAX_CORR, math.atan(K_LAT * lat)))
    # forward: steer right (-) when left of the line; reverse: the opposite
    ref = line_yaw + (corr if reverse else -corr)
    e = wrap(ref - pose[2])
    v = max(V_MIN, min(v_max, 0.6 * togo)) * max(0.0, math.cos(e))
    w = max(-W_MAX, min(W_MAX, K_YAW * e))
    return (-v if reverse else v), w, False, remaining, lat
