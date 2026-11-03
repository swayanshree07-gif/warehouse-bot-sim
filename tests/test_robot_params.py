"""Pure-Python checks of the robot geometry / kinematics (no Isaac Sim)."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))
import robot_params as P  # noqa: E402
import warehouse_layout as L  # noqa: E402


def test_wheel_speed_roundtrip():
    for v, w in [(0.5, 0.0), (0.0, 1.0), (0.3, -0.7), (-0.2, 0.4)]:
        v2, w2 = P.body_twist(*P.wheel_speeds(v, w))
        assert math.isclose(v, v2, abs_tol=1e-12) and math.isclose(w, w2, abs_tol=1e-12)


def test_straight_and_spin():
    left, right = P.wheel_speeds(0.5, 0.0)
    assert left == right == 0.5 / P.WHEEL_RADIUS
    left, right = P.wheel_speeds(0.0, 1.0)
    assert left == -right and right > 0   # CCW: right wheel forward


def test_geometry_consistent():
    # wheels outside the chassis, casters touch the floor, lift on top
    assert P.WHEEL_SEPARATION / 2 - P.WHEEL_WIDTH / 2 > P.CHASSIS_SIZE[1] / 2
    assert P.CASTER_RADIUS == P.CHASSIS_CLEARANCE
    assert P.CASTER_X + P.CASTER_RADIUS <= P.CHASSIS_SIZE[0] / 2
    assert P.WHEEL_RADIUS > P.CHASSIS_CLEARANCE


def test_robot_fits_aisles():
    # robot circumscribed diameter must fit the gap between shelf rows
    xs = [abs(x) for x, _ in P.footprint()]
    ys = [abs(y) for _, y in P.footprint()]
    diameter = 2 * math.hypot(max(xs), max(ys))
    sy = L.SHELF_SIZE[1]
    rows = sorted({s.y for s in L.SHELVES})
    gap = (rows[1] - sy / 2) - (rows[0] + sy / 2)
    assert diameter < gap - 1.0
