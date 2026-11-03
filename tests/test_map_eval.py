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
