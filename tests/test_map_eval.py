"""map_eval on synthetic maps: a perfect map covers ~100 %, an empty one 0 %."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))
import map_eval as M  # noqa: E402
import warehouse_layout as L  # noqa: E402


def _write_map(tmp_path, fill, start=(-2.0, 0.0, 0.0), res=0.05):
    # map frame = world shifted by the start pose (yaw 0 here)
    w, h = int(12 / res), int(10 / res)
    ox, oy = -6.0 - start[0], -5.0 - start[1]
    px = bytearray(w * h)
    for r in range(h):
        for c in range(w):
            mx, my = ox + (c + 0.5) * res, oy + (h - 1 - r + 0.5) * res
            px[r * w + c] = fill(mx + start[0], my + start[1])
    (tmp_path / "m.pgm").write_bytes(b"P5\n# test\n%d %d\n255\n" % (w, h) + bytes(px))
    (tmp_path / "m.yaml").write_text(
        f"image: m.pgm\nmode: trinary\nresolution: {res}\norigin: [{ox}, {oy}, 0]\nnegate: 0\n")
    return M.GridMap(str(tmp_path / "m.yaml"))


def _inside_room(x, y):
    return abs(x) < L.ROOM_X / 2 and abs(y) < L.ROOM_Y / 2


def test_perfect_map_full_coverage(tmp_path):
    g = _write_map(tmp_path, lambda x, y: 254 if _inside_room(x, y) else 0)
    cov, n = M.coverage(g, (-2.0, 0.0, 0.0))
    assert n > 20000 and cov > 0.999


def test_unknown_map_zero_coverage(tmp_path):
    g = _write_map(tmp_path, lambda x, y: 205)
    assert M.coverage(g, (-2.0, 0.0, 0.0))[0] == 0.0


def test_half_map(tmp_path):
    g = _write_map(tmp_path, lambda x, y: 254 if _inside_room(x, y) and x < 0 else 205)
    cov, _ = M.coverage(g, (-2.0, 0.0, 0.0))
    assert 0.4 < cov < 0.6


def _edge_fill(shift=0.0):
    """Independent GT rendering: a cell is occupied (0) if its centre is in an
    obstacle (or outside the room) and a 4-neighbour centre is free floor."""
    rects = M.G.obstacles_at_height(0.25)
    res = 0.05

    def free(x, y):
        return _inside_room(x, y) and not any(
            r[0] <= x <= r[2] and r[1] <= y <= r[3] for r in rects)

    def fill(x, y):
        x -= shift
        if free(x, y):
            return 254
        if any(free(x + dx, y + dy) for dx, dy in ((res, 0), (-res, 0), (0, res), (0, -res))):
            return 0
        return 205
    return fill


def test_iou_perfect_edges(tmp_path):
    g = _write_map(tmp_path, _edge_fill())
    m = M.occupied_iou(g, (-2.0, 0.0, 0.0), tol_cells=1)
    assert m["n_gt"] > 500 and m["iou"] > 0.9


def test_iou_empty_map_is_zero(tmp_path):
    g = _write_map(tmp_path, lambda x, y: 254)
    assert M.occupied_iou(g, (-2.0, 0.0, 0.0))["iou"] == 0.0


def test_iou_shifted_map_fails(tmp_path):
    # a map that is 0.4 m off in x must not pass the 0.7 gate
    g = _write_map(tmp_path, _edge_fill(shift=0.4))
    assert M.occupied_iou(g, (-2.0, 0.0, 0.0), tol_cells=2)["iou"] < 0.7


def test_visible_edges_exclude_wall_outsides():
    pts = M.visible_edge_points()
    assert all(abs(x) <= L.ROOM_X / 2 + 0.01 and abs(y) <= L.ROOM_Y / 2 + 0.01 for x, y in pts)
