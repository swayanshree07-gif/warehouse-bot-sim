"""Diff-drive robot geometry (pure Python, single source of truth).

Used by the USD builder (sim/robot.py) and later by the URDF/description,
Nav2 footprint and the cmd_vel -> wheel controller. Metres, kg, Z up,
x forward. base_link is the chassis centre, chassis_centre_z() above the
floor and directly above the wheel axle midpoint (Day 5: /odom tracks the
chassis body); base_footprint is the floor point below it (URDF, Day 8).

Decision (Day 3): own primitive robot instead of Nova Carter - no asset
download, cheap physics. Casters are frictionless spheres that are part of
the chassis body (no extra joints); only the two wheels are driven.
"""
import math

CHASSIS_SIZE = (0.50, 0.36, 0.14)   # x, y, z
CHASSIS_CLEARANCE = 0.04            # floor -> chassis bottom
CHASSIS_MASS = 12.0

WHEEL_RADIUS = 0.08
WHEEL_WIDTH = 0.04
WHEEL_SEPARATION = 0.44            # centre-to-centre (track width)
WHEEL_MASS = 1.0
WHEEL_MAX_TORQUE = 20.0            # N*m drive limit
WHEEL_DAMPING = 1.0e4              # velocity-drive gain (PhysX units)

CASTER_RADIUS = CHASSIS_CLEARANCE  # sphere touches the floor
CASTER_X = 0.19                    # front/back caster offset from axle

LIFT_SIZE = (0.30, 0.30, 0.02)
LIFT_TRAVEL = (0.0, 0.30)          # prismatic limits along z
LIFT_MASS = 1.0

# 2D lidar (PhysX raycast sensor, Day 6). Mounted front-top on a short post,
# in front of the lift plate (|x| <= 0.15) so the plate never blocks it while
# the lift is at or above the shelf height. Pose relative to base_link (= the
# chassis centre, z = 0.11 m above the floor) -> scan plane 0.25 m above floor,
# below the 0.35 m rack tops, so racks and walls are visible.
LASER_POS = (0.20, 0.0, 0.14)
LASER_RANGE = (0.12, 12.0)         # m (min, max)
LASER_RES_DEG = 1.0                # 360 rays
LASER_RATE_HZ = 10.0               # published every N physics ticks

MAX_LINEAR_SPEED = 0.8             # m/s (used by Nav2 later)
MAX_ANGULAR_SPEED = 2.0            # rad/s


def chassis_centre_z():
    return CHASSIS_CLEARANCE + CHASSIS_SIZE[2] / 2


def chassis_top_z():
    return CHASSIS_CLEARANCE + CHASSIS_SIZE[2]


def laser_height():
    return chassis_centre_z() + LASER_POS[2]


def laser_angles_deg():
    """Beam angles (deg, laser frame) in publish order: -180 .. 180-res."""
    n = int(round(360.0 / LASER_RES_DEG))
    return [-180.0 + i * LASER_RES_DEG for i in range(n)]


def footprint():
    """Rectangular footprint polygon [(x, y)] around base_link (for Nav2)."""
    hx = CHASSIS_SIZE[0] / 2
    hy = WHEEL_SEPARATION / 2 + WHEEL_WIDTH / 2
    return [(hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy)]


def wheel_speeds(v, w):
    """Unicycle (v m/s, w rad/s) -> (left, right) wheel angular speed rad/s."""
    half = WHEEL_SEPARATION / 2
    return (v - w * half) / WHEEL_RADIUS, (v + w * half) / WHEEL_RADIUS


def body_twist(w_left, w_right):
    """Inverse of wheel_speeds: wheel rad/s -> (v, w)."""
    vl, vr = w_left * WHEEL_RADIUS, w_right * WHEEL_RADIUS
    return (vl + vr) / 2, (vr - vl) / WHEEL_SEPARATION


def rad_to_deg(x):
    return x * 180.0 / math.pi
