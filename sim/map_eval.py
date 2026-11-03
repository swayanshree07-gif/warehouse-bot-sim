"""Compare a saved occupancy map (map_saver PGM + YAML) with the scene's
ground truth (pure Python, Days 10-11).

  python3 sim/map_eval.py out/maps/warehouse.yaml [--start=-2,0,0] [--min-coverage 0.8]

The map frame is slam_toolbox's, which starts at the robot's first odom pose,
i.e. the sim start pose (x, y, yaw) in world coordinates. Coverage = fraction
of ground-truth free cells (room interior minus racks, eroded by MARGIN so
edge effects don't count) that are free (254) in the trinary PGM.
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geometry2d as G  # noqa: E402
import warehouse_layout as L  # noqa: E402

MARGIN = 0.15   # m kept clear of every obstacle edge when counting GT free cells


def load_yaml(path):
    """Tiny parser for map_saver's flat YAML (no PyYAML needed)."""
    out = {}
    for line in open(path):
        if ":" not in line:
            continue
        k, v = (s.strip() for s in line.split(":", 1))
        if v.startswith("["):
            out[k] = [float(x) for x in v.strip("[]").split(",")]
        else:
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


def load_pgm(path):
    """Binary P5 PGM -> (width, height, bytes) with row 0 = top of image."""
    data = open(path, "rb").read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    assert tokens[0] == b"P5", "only binary PGM supported"
    w, h = int(tokens[1]), int(tokens[2])
    pos += 1
    return w, h, data[pos:pos + w * h]


class GridMap:
    def __init__(self, yaml_path):
        meta = load_yaml(yaml_path)
        img = meta["image"]
        if not os.path.isabs(img):
            img = os.path.join(os.path.dirname(yaml_path), img)
        self.w, self.h, self.px = load_pgm(img)
        self.res = meta["resolution"]
        self.ox, self.oy = meta["origin"][0], meta["origin"][1]

    def value_at(self, mx, my):
        """Pixel at map-frame point, or None outside the image."""
        c = int(math.floor((mx - self.ox) / self.res))
        r = int(math.floor((my - self.oy) / self.res))
        if not (0 <= c < self.w and 0 <= r < self.h):
            return None
        return self.px[(self.h - 1 - r) * self.w + c]


def world_to_map(x, y, start):
    sx, sy, syaw = start
    dx, dy = x - sx, y - sy
    c, s = math.cos(-syaw), math.sin(-syaw)
    return c * dx - s * dy, s * dx + c * dy


def gt_free_points(step, z=0.25, margin=MARGIN):
    """World (x, y) cell centres that are free in the scene at height z."""
    rects = [r for r in G.obstacles_at_height(z)]
    hx, hy = L.ROOM_X / 2, L.ROOM_Y / 2
    pts = []
    n_x, n_y = int(L.ROOM_X / step), int(L.ROOM_Y / step)
    for i in range(n_x):
        for j in range(n_y):
            x, y = -hx + (i + 0.5) * step, -hy + (j + 0.5) * step
            if abs(x) > hx - margin or abs(y) > hy - margin:
                continue
            if any(r[0] - margin <= x <= r[2] + margin and r[1] - margin <= y <= r[3] + margin for r in rects):
                continue
            pts.append((x, y))
    return pts


def coverage(grid, start, step=0.05):
    pts = gt_free_points(step)
    free = sum(1 for x, y in pts if grid.value_at(*world_to_map(x, y, start)) == 254)
    return free / len(pts), len(pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml")
    ap.add_argument("--start", default="-2,0,0")
    ap.add_argument("--min-coverage", type=float, default=0.8)
    a = ap.parse_args()
    sx, sy, syaw = (float(v) for v in a.start.split(","))
    grid = GridMap(a.yaml)
    cov, n = coverage(grid, (sx, sy, math.radians(syaw)))
    ok = cov >= a.min_coverage
    print(f"MAP_COVERAGE {cov:.3f} of {n} ground-truth free cells (need >= {a.min_coverage}) -> {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
