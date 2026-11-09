import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from fleet_layout import FLEET_STARTS, frames, prim_path, topics  # noqa: E402
import warehouse_layout as L  # noqa: E402


def test_three_unique_namespaces():
    assert sorted(FLEET_STARTS) == ["robot1", "robot2", "robot3"]
    assert len({prim_path(n) for n in FLEET_STARTS}) == 3
    assert frames("robot2") == ("robot2/odom", "robot2/base_link", "robot2/laser")
    assert topics("robot1") == ["/robot1/cmd_vel", "/robot1/odom", "/robot1/scan"]


def test_starts_free_and_separated():
    pts = list(FLEET_STARTS.values())
    for i, a in enumerate(pts):
        assert abs(a[0]) < L.ROOM_X / 2 - 0.5 and abs(a[1]) < L.ROOM_Y / 2 - 0.5
        for s in L.SHELVES:  # clear of every rack by > 0.5 m
            assert abs(a[1] - s.y) > L.SHELF_SIZE[1] / 2 + 0.5 or abs(a[0] - s.x) > L.SHELF_SIZE[0] / 2 + 0.5
        for b in pts[i + 1:]:
            assert math.hypot(a[0] - b[0], a[1] - b[1]) >= 2.0
