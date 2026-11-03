"""Ground-truth 2D raycast used by the /scan check (no Isaac Sim needed)."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))
import geometry2d as G  # noqa: E402
import robot_params as P  # noqa: E402
import warehouse_layout as L  # noqa: E402


def test_scan_plane_sees_walls_and_racks_not_boxes():
    rects = G.obstacles_at_height(P.laser_height())
    assert len(rects) == 4 + len(L.SHELVES)  # boxes sit above the rack tops


def test_wall_distances_from_centre():
    rects = G.obstacles_at_height(0.25)
    assert math.isclose(G.raycast(0, 0, 0.0, rects), L.ROOM_X / 2)
    assert math.isclose(G.raycast(0, 0, math.pi, rects), L.ROOM_X / 2)
    # +y from the centre passes between racks A/B (they end at |x| = 1.75)
    assert math.isclose(G.raycast(0, 0, math.pi / 2, rects), L.ROOM_Y / 2)


def test_rack_hit():
    rects = G.obstacles_at_height(0.25)
    # from (-2.5, 0) looking +y: rack A front face at y = 2.0 - 0.2
    assert math.isclose(G.raycast(-2.5, 0, math.pi / 2, rects), 1.8)


def test_max_range_and_inside():
    rects = [(-1, -1, 1, 1)]
    assert G.raycast(5, 5, 0.0, rects, 3.0) == 3.0
    assert G.raycast(0, 0, 0.0, rects) == 0.0


def test_laser_mount_clear_of_lift_and_racks():
    assert P.LASER_POS[0] > P.LIFT_SIZE[0] / 2
    assert P.LASER_POS[0] < P.CHASSIS_SIZE[0] / 2
    assert P.chassis_top_z() < P.laser_height() < L.SHELF_SIZE[2]
    assert len(P.laser_angles_deg()) == 360
