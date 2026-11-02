"""Pure-Python checks of the warehouse layout (no Isaac Sim needed).
Run: python3 -m pytest -q tests/
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))
import warehouse_layout as L  # noqa: E402


def _rect(cx, cy, sx, sy):
    return (cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)


def _overlap(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def test_box_count():
    assert len(L.BOXES) == 10
    assert len({b.name for b in L.BOXES}) == 10


def test_shelves_inside_room_with_clearance():
    sx, sy, _ = L.SHELF_SIZE
    for s in L.SHELVES:
        r = _rect(s.x, s.y, sx, sy)
        assert r[0] > -L.ROOM_X / 2 + 1.0 and r[2] < L.ROOM_X / 2 - 1.0
        assert r[1] > -L.ROOM_Y / 2 + 1.0 and r[3] < L.ROOM_Y / 2 - 1.0


def test_shelves_do_not_overlap_and_aisles_wide():
    sx, sy, _ = L.SHELF_SIZE
    rects = [_rect(s.x, s.y, sx + 1.5, sy + 1.5) for s in L.SHELVES]  # 0.75 m margin each side
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            assert not _overlap(rects[i], rects[j])


def test_boxes_sit_on_their_shelf():
    shelves = {s.name: s for s in L.SHELVES}
    sx, sy, sz = L.SHELF_SIZE
    for b in L.BOXES:
        s = shelves[b.shelf]
        assert abs(b.z - (sz + L.BOX_SIZE / 2)) < 1e-9
        assert abs(b.x - s.x) + L.BOX_SIZE / 2 <= sx / 2
        assert abs(b.y - s.y) + L.BOX_SIZE / 2 <= sy / 2


def test_boxes_do_not_overlap():
    rects = [_rect(b.x, b.y, L.BOX_SIZE, L.BOX_SIZE) for b in L.BOXES]
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            assert not _overlap(rects[i], rects[j])


def test_walls_enclose_room():
    names = {w[0] for w in L.walls()}
    assert names == {"wall_north", "wall_south", "wall_east", "wall_west"}
