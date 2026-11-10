"""Day 16: traffic manager for the fleet (pure Python, no ROS).

Model
-----
* ``Roadmap``: a coarse 4-connected grid (default 0.5 m cells) built from the
  saved occupancy map (PGM + YAML). A cell is free only if every map pixel
  within ``inflate`` metres of it is known-free, so a robot centred in a cell
  clears walls/shelves.
* ``TrafficManager``: robots move cell-to-cell in discrete ticks. Each robot
  occupies exactly one cell. Rules per tick (robots processed by priority,
  lower number = higher priority):
    - a robot may enter its next cell only if no robot occupies it now and no
      higher-priority robot has already claimed it this tick (cell
      reservation). Requiring the cell to be *empty now* also forbids swaps
      and edge conflicts, so two robots can never share a cell or pass
      through each other.
    - a blocked robot waits. The wait-for graph (robot -> robot sitting on its
      next cell) is checked every tick; a cycle (head-on swap, 3-robot ring,
      narrow aisle) is a deadlock.
    - deadlock resolution = priority yield: the lowest-priority robot in the
      cycle retreats (BFS through empty cells) to the nearest cell that is not
      on the other cycle robots' remaining paths, then holds there until those
      robots' remaining paths no longer cross its own route to the goal (or
      they finish), and then replans.
  A robot that waits ``stall_limit`` ticks on a non-cyclic blocker (e.g. a
  parked robot standing on its path) replans around occupied cells.
"""
import heapq
import os
from collections import deque

FREE, OCC = 0, 1


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b"P5":
        raise ValueError("only binary PGM (P5) supported")
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError("16-bit PGM not supported")
    raw = data[i + 1:i + 1 + w * h]
    return w, h, [raw[r * w:(r + 1) * w] for r in range(h)]


def read_map_yaml(path):
    meta = {}
    with open(path) as f:
        for line in f:
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    origin = [float(x) for x in meta["origin"].strip("[]").split(",")]
    return {
        "image": meta["image"],
        "resolution": float(meta["resolution"]),
        "origin": origin,
        "negate": int(meta.get("negate", 0)),
        "free_thresh": float(meta.get("free_thresh", 0.196)),
    }


class Roadmap:
    """4-connected grid of cells; ``free`` is a set of (col, row)."""

    def __init__(self, cols, rows, free, cell=0.5, origin=(0.0, 0.0)):
        self.cols, self.rows, self.cell = cols, rows, cell
        self.origin = origin
        self.free = set(free)

    @classmethod
    def from_ascii(cls, text, cell=1.0):
        """Rows top->bottom; '#' blocked, anything else free. (0,0) = bottom-left."""
        lines = text.strip("\n").splitlines()
        rows = len(lines)
        cols = max(len(ln) for ln in lines)
        free = set()
        for r, ln in enumerate(lines):
            for c, ch in enumerate(ln):
                if ch != "#":
                    free.add((c, rows - 1 - r))
        return cls(cols, rows, free, cell)

    @classmethod
    def from_map(cls, yaml_path, cell=0.5, inflate=0.3):
        meta = read_map_yaml(yaml_path)
        w, h, img = read_pgm(os.path.join(os.path.dirname(yaml_path), meta["image"]))
        res = meta["resolution"]
        ox, oy = meta["origin"][0], meta["origin"][1]
        thresh = 255 * (1.0 - meta["free_thresh"])

        def px_free(x, y):  # x,y pixel coords with y up
            v = img[h - 1 - y][x]
            if meta["negate"]:
                v = 255 - v
            return v > thresh

        cols, rows = int(w * res / cell), int(h * res / cell)
        rad = int(round(inflate / res))
        free = set()
        for c in range(cols):
            for r in range(rows):
                cx = int(((c + 0.5) * cell) / res)
                cy = int(((r + 0.5) * cell) / res)
                ok = True
                for dx in range(-rad, rad + 1):
                    for dy in range(-rad, rad + 1):
                        if dx * dx + dy * dy > rad * rad:
                            continue
                        x, y = cx + dx, cy + dy
                        if not (0 <= x < w and 0 <= y < h) or not px_free(x, y):
                            ok = False
                            break
                    if not ok:
                        break
                if ok:
                    free.add((c, r))
        return cls(cols, rows, free, cell, (ox, oy))

    def to_cell(self, x, y):
        return (int((x - self.origin[0]) // self.cell), int((y - self.origin[1]) // self.cell))

    def to_world(self, cell):
        return (self.origin[0] + (cell[0] + 0.5) * self.cell,
                self.origin[1] + (cell[1] + 0.5) * self.cell)

    def nearest_free(self, cell):
        if cell in self.free:
            return cell
        return min(self.free, key=lambda f: (abs(f[0] - cell[0]) + abs(f[1] - cell[1]), f))

    def neighbors(self, cell):
        c, r = cell
        for n in ((c + 1, r), (c - 1, r), (c, r + 1), (c, r - 1)):
            if n in self.free:
                yield n

    def plan(self, start, goal, blocked=()):
        """A* path [start, ..., goal] or None. ``blocked`` cells are avoided."""
        blocked = set(blocked) - {start, goal}
        if goal not in self.free:
            return None
        h = lambda n: abs(n[0] - goal[0]) + abs(n[1] - goal[1])  # noqa: E731
        openq = [(h(start), 0, start)]
        came, g = {start: None}, {start: 0}
        while openq:
            _, cost, cur = heapq.heappop(openq)
            if cur == goal:
                path = []
                while cur is not None:
                    path.append(cur)
                    cur = came[cur]
                return path[::-1]
            if cost > g[cur]:
                continue
            for n in self.neighbors(cur):
                if n in blocked:
                    continue
                if cost + 1 < g.get(n, 1 << 30):
                    g[n] = cost + 1
                    came[n] = cur
                    heapq.heappush(openq, (cost + 1 + h(n), cost + 1, n))
        return None

    def path_length(self, a, b):
        """Roadmap distance in metres between world points (for fleet.py)."""
        p = self.plan(self.nearest_free(self.to_cell(*a[:2])),
                      self.nearest_free(self.to_cell(*b[:2])))
        return float("inf") if p is None else (len(p) - 1) * self.cell


class Agent:
    def __init__(self, name, cell, priority):
        self.name, self.cell, self.priority = name, cell, priority
        self.goal = None
        self.path = []          # remaining cells to visit (excludes current)
        self.hold_for = set()   # names we yield to; waiting at a refuge
        self.final_route = []   # route to goal used to decide release
        self.waited = 0
        self.wait_ticks = 0
        self.moves = 0

    @property
    def done(self):
        return self.goal is None or (self.cell == self.goal and not self.path)


class TrafficManager:
    def __init__(self, roadmap, stall_limit=6):
        self.rm = roadmap
        self.agents = {}
        self.stall_limit = stall_limit
        self.tick_no = 0
        self.deadlocks = 0
        self.yields = 0
        self.pinned = set()     # Day 25: robots docked at a shelf; never asked to step aside
        self.ghosts = {}        # Day 25: extra cell -> robot name (docked robot overhangs it)

    # ---- setup -------------------------------------------------------
    def add_robot(self, name, cell, priority=None):
        if cell not in self.rm.free:
            raise ValueError(f"{name}: start cell {cell} not free")
        if any(a.cell == cell for a in self.agents.values()):
            raise ValueError(f"{name}: start cell {cell} already occupied")
        prio = len(self.agents) if priority is None else priority
        self.agents[name] = Agent(name, cell, prio)

    def set_goal(self, name, goal):
        a = self.agents[name]
        if goal not in self.rm.free:
            raise ValueError(f"{name}: goal {goal} not free")
        a.goal, a.hold_for = goal, set()
        p = self.rm.plan(a.cell, goal)
        if p is None:
            raise ValueError(f"{name}: no path {a.cell}->{goal}")
        a.path = p[1:]

    def occupied(self):
        occ = dict(self.ghosts)
        occ.update({a.cell: a.name for a in self.agents.values()})
        return occ

    def all_done(self):
        return all(a.done and not a.hold_for for a in self.agents.values())

    # ---- deadlock handling ------------------------------------------
    def _wait_for(self):
        occ = self.occupied()
        wf = {}
        for a in self.agents.values():
            if a.path and not a.hold_for:
                other = occ.get(a.path[0])
                if other and other != a.name:
                    wf[a.name] = other
        return wf

    def _find_cycle(self, wf):
        for start in sorted(wf):
            seen, cur = [], start
            while cur in wf and cur not in seen:
                seen.append(cur)
                cur = wf[cur]
            if cur in seen:
                return seen[seen.index(cur):]
        return None

    def _refuge_route(self, mover, avoid):
        """BFS through empty cells to the nearest cell not in ``avoid``."""
        occ = set(self.occupied()) - {mover.cell}
        q, prev = deque([mover.cell]), {mover.cell: None}
        while q:
            cur = q.popleft()
            if cur not in avoid:
                route = []
                while cur != mover.cell:
                    route.append(cur)
                    cur = prev[cur]
                return route[::-1]
            for n in self.rm.neighbors(cur):
                if n not in prev and n not in occ:
                    prev[n] = cur
                    q.append(n)
        return None

    def _step_aside(self, mover, others):
        if mover.name in self.pinned:
            return False
        avoid = set()
        for o in others:
            avoid |= {o.cell, *o.path}
        route = self._refuge_route(mover, avoid)
        if route is None:
            return False
        mover.path = route
        mover.hold_for = {o.name for o in others}
        mover.waited = 0
        self.yields += 1
        return True

    def _yield(self, cycle):
        """Priority yield: the lowest-priority robot in ``cycle`` that can
        reach a refuge off the others' paths steps aside."""
        cyc = sorted((self.agents[n] for n in cycle),
                     key=lambda a: (a.priority, a.name), reverse=True)
        for mover in cyc:
            if self._step_aside(mover, [a for a in cyc if a is not mover]):
                return True
        return False

    def _maybe_release(self, a):
        if not a.hold_for or a.path:
            return
        route = self.rm.plan(a.cell, a.goal) or []
        still = False
        for n in a.hold_for:
            o = self.agents[n]
            if o.done:
                continue
            if set(route) & ({o.cell} | set(o.path)):
                still = True
        a.hold_for = set() if not still else a.hold_for
        if not still:
            a.path = route[1:]

    # ---- main step ---------------------------------------------------
    def tick(self):
        self.tick_no += 1
        for a in self.agents.values():
            self._maybe_release(a)
        cyc = self._find_cycle(self._wait_for())
        if cyc:
            self.deadlocks += 1
            self._yield(cyc)
        occ = self.occupied()
        claimed = set()
        moved = []
        for a in sorted(self.agents.values(), key=lambda a: (a.priority, a.name)):
            if not a.path:
                continue
            nxt = a.path[0]
            if nxt not in occ and nxt not in claimed:
                claimed.add(nxt)
                moved.append((a, nxt))
                a.waited = 0
            else:
                claimed.add(a.cell)
                a.waited += 1
                a.wait_ticks += 1
                if a.waited >= self.stall_limit and not a.hold_for:
                    self._unstall(a, occ)
                    a.waited = 0
        for a, nxt in moved:
            a.cell = nxt
            a.path.pop(0)
            a.moves += 1
        cells = [a.cell for a in self.agents.values()]
        assert len(cells) == len(set(cells)), "two robots in one cell"
        return {a.name: a.cell for a in self.agents.values()}

    def _unstall(self, a, occ):
        """``a`` waited too long on a non-cyclic blocker (e.g. a parked robot,
        or a chain ending in one). First try to replan around occupied cells;
        otherwise the lowest-priority robot standing on a waiting robot's path
        steps aside to a refuge off all waiting robots' paths and comes back
        once they have passed."""
        p = self.rm.plan(a.cell, a.goal, blocked=set(occ) - {a.cell})
        if p:
            a.path = p[1:]
            return
        waiters = [w for w in self.agents.values()
                   if w.path and not w.hold_for and w.waited > 0]
        pressure = set()
        for w in waiters:
            pressure |= set(w.path)
        # prefer parked robots, then lowest priority
        cands = sorted((c for c in self.agents.values()
                        if c.cell in pressure and not c.hold_for),
                       key=lambda c: (not c.done, -c.priority, c.name))
        for c in cands:
            others = [w for w in waiters if w is not c]
            if self._step_aside(c, others):
                if c.goal is None:
                    c.goal = c.cell  # return to where it was parked
                return

    def run(self, max_ticks=500):
        """Tick until all robots are done. Returns ticks used, or raises."""
        history = []
        for _ in range(max_ticks):
            if self.all_done():
                return self.tick_no, history
            history.append(self.tick())
        raise RuntimeError(f"not finished after {max_ticks} ticks: "
                           + str({n: (a.cell, a.goal) for n, a in self.agents.items()}))
