"""Light 2D fleet simulator (Day 17) - no GPU, pure Python.

Used to test fleet logic quickly and as the fallback simulator when a
3-robot Isaac scene does not fit in 8 GB VRAM.

Model:
- The warehouse is the `traffic.Roadmap` grid (0.5 m cells from the saved map).
- Each robot is kinematic: it moves one cell per traffic tick (`tick_s`
  seconds, i.e. speed = cell / tick_s) and is linearly interpolated between
  cell centres, so separation is measured on continuous positions.
- `FleetManager` assigns tasks (nearest free robot by roadmap path length);
  `TrafficManager` reserves cells and resolves deadlocks.
- Task phases: go to pick approach cell -> pick (pick_s) -> go to drop
  approach cell -> drop (drop_s) -> done. Robots then park where they are
  (parked robots step aside when in the way; see traffic.py).
- A floor drop zone `zone_E` (3 slots) is added to the light sim's registry
  copy so that 3 tasks can run at once (the generated registry has only 2
  free shelf slots). It is not part of the Isaac scene yet.
- Scenario tasks are submitted lazily: a task whose target has no free,
  unclaimed slot yet waits in a pending list and is retried every tick.
"""
import math

from .fleet import FleetManager
from .registry import TaskError
from .traffic import TrafficManager

DROP_ZONE = {
    "center": [2.0, 3.1],
    "yaw_deg": 0.0,
    "size": [1.5, 0.4, 0.0],
    "access_side": "-y",
    "slots": [
        {"index": i, "center": [x, 3.1, 0.1], "approach": [x, 2.6, 90.0],
         "dock": [x, 2.8, 90.0]}
        for i, x in enumerate((1.5, 2.0, 2.5))
    ],
}

# 9 tasks for 3 robots; each target uses an auto slot.
DEFAULT_SCENARIO = [
    ("box_00", "zone_E"), ("box_04", "zone_E"), ("box_08", "zone_E"),
    ("box_01", "shelf_D"), ("box_06", "shelf_D"),
    ("box_03", "shelf_A"), ("box_07", "shelf_A"), ("box_09", "shelf_B"),
    ("box_02", "shelf_C"),
]


class LightSim:
    def __init__(self, roadmap, registry, starts, tick_s=1.0, pick_s=4.0,
                 drop_s=4.0, add_drop_zone=True, substeps=5):
        self.rm = roadmap
        reg = registry.copy()
        if add_drop_zone and "zone_E" not in reg.shelves:
            reg.shelves["zone_E"] = DROP_ZONE
        self.fleet = FleetManager(reg, distance_fn=roadmap.path_length)
        self.tm = TrafficManager(roadmap)
        self.tick_s, self.substeps = tick_s, substeps
        self.pick_ticks = max(1, round(pick_s / tick_s))
        self.drop_ticks = max(1, round(drop_s / tick_s))
        self.t = 0.0
        self.phase = {}       # name -> (phase, task_id, ticks_left)
        self.pending = []     # (box, shelf) not yet accepted by the fleet
        self.rejected = []
        self.min_sep = math.inf
        self.collisions = 0
        self.events = []
        for name, xy in starts.items():
            cell = roadmap.nearest_free(roadmap.to_cell(*xy))
            self.tm.add_robot(name, cell)
            self.fleet.add_robot(name, (*roadmap.to_world(cell), 0.0))
            self.phase[name] = ("idle", -1, 0)

    # ---- helpers --------------------------------------------------------
    def cell_of(self, pose):
        return self.rm.nearest_free(self.rm.to_cell(*pose[:2]))

    def pos(self, name):
        return self.rm.to_world(self.tm.agents[name].cell)

    def _log(self, msg):
        self.events.append((round(self.t, 2), msg))

    def submit(self, box, shelf):
        self.pending.append((box, shelf))

    def _try_submit(self):
        left = []
        for box, shelf in self.pending:
            try:
                t = self.fleet.submit(box, shelf, now=self.t)
                self._log(f"task {t.id} {box}->{shelf}[{t.plan['dst_slot']}] queued")
            except TaskError as e:
                if "no free" in str(e) or "claimed" in str(e) or "active task" in str(e):
                    left.append((box, shelf))   # retry later
                else:
                    self.rejected.append((box, shelf, str(e)))
        self.pending = left

    def _set_goal(self, name, pose):
        a = self.tm.agents[name]
        a.hold_for = set()
        self.tm.set_goal(name, self.cell_of(pose))

    # ---- step -----------------------------------------------------------
    def step(self):
        self._try_submit()
        for task, robot in self.fleet.dispatch(now=self.t):
            self.phase[robot.name] = ("to_pick", task.id, 0)
            self._set_goal(robot.name, task.pick_pose)
            self._log(f"task {task.id} -> {robot.name}")
        before = {n: self.pos(n) for n in self.tm.agents}
        self.tm.tick()
        after = {n: self.pos(n) for n in self.tm.agents}
        self._check_separation(before, after)
        self.t += self.tick_s
        for name, (ph, tid, left) in list(self.phase.items()):
            a = self.tm.agents[name]
            arrived = a.done and not a.hold_for
            task = self.fleet.tasks.get(tid)
            if ph == "to_pick" and arrived:
                self.phase[name] = ("picking", tid, self.pick_ticks)
            elif ph == "picking":
                if left <= 1:
                    self.phase[name] = ("to_drop", tid, 0)
                    self._set_goal(name, task.drop_pose)
                    self._log(f"{name} picked {task.box_id}")
                else:
                    self.phase[name] = (ph, tid, left - 1)
            elif ph == "to_drop" and arrived:
                self.phase[name] = ("dropping", tid, self.drop_ticks)
            elif ph == "dropping":
                if left <= 1:
                    self.fleet.finish(tid, ok=True, pose=(*after[name], 0.0), now=self.t)
                    self.phase[name] = ("idle", -1, 0)
                    a.goal = None
                    self._log(f"{name} done task {tid} ({task.box_id})")
                else:
                    self.phase[name] = (ph, tid, left - 1)
            elif ph == "idle":
                self.fleet.robots[name].pose = (*after[name], 0.0)

    def _check_separation(self, before, after):
        names = sorted(before)
        for k in range(self.substeps + 1):
            f = k / self.substeps
            p = {n: (before[n][0] + f * (after[n][0] - before[n][0]),
                     before[n][1] + f * (after[n][1] - before[n][1])) for n in names}
            for i, a in enumerate(names):
                for b in names[i + 1:]:
                    d = math.hypot(p[a][0] - p[b][0], p[a][1] - p[b][1])
                    self.min_sep = min(self.min_sep, d)
                    if d < 1e-6:
                        self.collisions += 1

    def finished(self):
        return (not self.pending and not self.fleet.queue
                and all(ph == "idle" for ph, _, _ in self.phase.values()))

    def run(self, max_time=3600.0):
        while not self.finished():
            if self.t >= max_time:
                raise RuntimeError(f"scenario not finished after {max_time}s: "
                                   f"{self.fleet.status()} pending={self.pending}")
            self.step()
        return self.report()

    def report(self):
        tasks = list(self.fleet.tasks.values())
        return {
            "sim_time_s": self.t,
            "tasks_done": sum(t.state == "done" for t in tasks),
            "tasks_failed": sum(t.state == "failed" for t in tasks),
            "rejected": len(self.rejected),
            "min_separation_m": round(self.min_sep, 3),
            "collisions": self.collisions,
            "deadlocks_resolved": self.tm.deadlocks,
            "per_robot_tasks": {n: sum(t.robot == n for t in tasks) for n in self.fleet.robots},
            "boxes": {b: list(v) for b, v in sorted(self.fleet.registry.boxes.items())},
        }
