"""Fleet executor for real (continuous) robots (Day 25), pure Python.

Runs FleetManager tasks on robots that report continuous map-frame poses and
take (v, w) commands - the Isaac fleet. Same task semantics as LightSim
(pending -> queued -> assigned -> done) and the same status attributes, so
`fleet_manager.status_dict()` works on it unchanged.

Per robot phases:
  idle -> to_pick (traffic) -> dock_in -> pick (gripper action) -> dock_out
       -> to_drop (traffic) -> dock_in -> drop (gripper action) -> dock_out -> idle
Traffic phases are lockstep like FleetFollower: the TrafficManager only ticks
once every robot that is in traffic sits at its target cell centre. Robots in
dock_in/pick/drop/dock_out are "pinned" (excluded from the arrival check and
never asked to step aside); they keep their reserved approach cell.
Docking: drive to the slot approach pose, turn to the slot yaw, creep straight
to the dock pose, do the gripper action, reverse straight back to the approach.
Gripper actions are asynchronous: the caller pops `requests` ((robot, kind,
box) with kind "pick"|"drop"), runs them and calls `action_done(robot, ok)`.
"""
import math

from .fleet import FleetManager
from .metrics import compute_metrics
from .path_follower import cmd_to, wrap
from .rack_align import compose
from .line_track import errors, track_cmd
from .registry import TaskError
from .traffic import TrafficManager

TRAFFIC = ("idle", "to_pick", "to_drop")


def dock_cmd(pose, target, yaw, v_max=0.12, tol=0.005):
    """Track the line through `target` with direction `yaw` (line_track, Day 20),
    forward if the target is ahead, else in reverse. Returns (v, w, arrived)."""
    remaining, _ = errors(pose, target, yaw)
    v, w, ok, _, _ = track_cmd(pose, target, yaw, reverse=remaining < 0, v_max=v_max, tol=tol)
    return v, w, ok


class FleetExecutor:
    def __init__(self, roadmap, registry, poses, tol=0.1, yaw_tol=0.03, measure_fn=None,
                 settle_s=0.6, backup_lat=0.02, backup_m=0.75, place_error_fn=None):
        """measure_fn(name, shelf_dict, slot_dict) -> dock pose (x, y, yaw) in
        the robot's base frame measured with the lidar (rack_align), or None.
        place_error_fn(name, box_id, slot_dict) -> (dx, dy) map-frame offset of
        the held box from the slot centre, or None."""
        self.measure_fn, self.settle_s = measure_fn, settle_s
        self.backup_lat, self.backup_m = backup_lat, backup_m
        self.dock_goal, self.settle, self._dt = {}, {}, 0.0
        self.failed_out, self.fixed = set(), {}   # fixed: name -> re-docks done
        self.place_error_fn = place_error_fn
        """poses: name -> (x, y, yaw_rad) current map-frame poses."""
        self.rm = roadmap
        self.fleet = FleetManager(registry.copy(), distance_fn=roadmap.path_length)
        self.tm = TrafficManager(roadmap)
        self.tol, self.yaw_tol = tol, yaw_tol
        self.t = 0.0
        self.poses = dict(poses)
        self.phase = {}         # name -> (phase, task_id, sub)
        self.pending, self.rejected, self.events, self.requests = [], [], [], []
        self.min_sep, self.collisions = math.inf, 0
        self.robot_time = {}
        for n, (x, y, _) in poses.items():
            cell = roadmap.nearest_free(roadmap.to_cell(x, y))
            self.tm.add_robot(n, cell)
            self.fleet.add_robot(n, (x, y, 0.0))
            self.phase[n] = ("idle", -1, 0)
            self.robot_time[n] = {"idle": 0.0, "busy": 0.0, "traffic_wait": 0.0}
        self.target = {n: roadmap.to_world(a.cell) for n, a in self.tm.agents.items()}
        # Day 26: cells in front of shelf slots (approach cells + neighbours).
        # Robots park off these after each task so a carrying robot (box sticks
        # ~0.5 m out in front) never has to turn next to a parked robot.
        self.shelf_front = set()
        for sh in registry.shelves.values():
            for sl in sh["slots"]:
                c = roadmap.nearest_free(roadmap.to_cell(*sl["approach"][:2]))
                self.shelf_front |= {c, *roadmap.neighbors(c)}
        self.progress = {n: (self.t, tuple(p[:2])) for n, p in poses.items()}
        self.stalls = 0
        # Day 28: lockstep watchdog. Traffic ticks only when every robot is
        # within `tol` of its cell centre; in Isaac a robot can stop ~0.14 m
        # off it (wheels/contacts) and the whole fleet froze. A robot that has
        # stood still `settle_s` s inside its target cell counts as arrived.
        self.still = {n: (self.t, tuple(p)) for n, p in poses.items()}
        self.settle_s, self.settle_moved = 5.0, 0.02
        self.settled = 0
        self._settled_logged = set()
        # Day 27: shelf lock. Neighbour slots are 0.5 m apart, less than a
        # robot plus a carried box, so only one robot at a time may travel to,
        # dock at or undock from a shelf. The lock is taken when a leg starts
        # and released once the robot has left that shelf's front cells.
        self.shelf_cells = {}
        for sid, sh in registry.shelves.items():
            cells = set()
            for sl in sh["slots"]:
                c = roadmap.nearest_free(roadmap.to_cell(*sl["approach"][:2]))
                cells |= {c, *roadmap.neighbors(c)}
            self.shelf_cells[sid] = cells
        self.lock = {}          # shelf id -> robot name
        self.leaving = {}       # robot name -> shelf id it still holds while leaving
        self.lock_waits = 0
        # Day 27: on a drop re-dock, re-measure the rack with the lidar after
        # backing up (instead of a blind shift) and apply the accumulated
        # held-box offset to the fresh measurement.
        self.box_off, self.remeasure = {}, set()

    # ---- status helpers (same names as LightSim) -------------------------
    def pos(self, name):
        return self.poses[name][:2]

    def _log(self, msg):
        self.events.append((round(self.t, 2), msg))

    def submit(self, box, shelf):
        self.pending.append((box, shelf))

    def finished(self):
        return (not self.pending and not self.fleet.queue
                and all(ph == "idle" for ph, _, _ in self.phase.values()))

    def busy(self):
        return not self.finished()

    def _try_submit(self):
        left = []
        for box, shelf in self.pending:
            try:
                t = self.fleet.submit(box, shelf, now=self.t)
                self._log(f"task {t.id} {box}->{shelf}[{t.plan['dst_slot']}] queued")
            except TaskError as e:
                if "no free" in str(e) or "claimed" in str(e) or "active task" in str(e):
                    left.append((box, shelf))
                else:
                    self.rejected.append((box, shelf, str(e)))
                    self._log(f"rejected {box}->{shelf}: {e}")
        self.pending = left

    def _cell(self, pose):
        return self.rm.nearest_free(self.rm.to_cell(*pose[:2]))

    def _ghost_cell(self, name, point):
        """Neighbour cell the robot overhangs when it leaves its cell centre
        for `point` (slot approach/dock lie between cell centres), or None."""
        c = self.tm.agents[name].cell
        cx, cy = self.rm.to_world(c)
        dx, dy = point[0] - cx, point[1] - cy
        if max(abs(dx), abs(dy)) < 0.1:
            return None
        g = (c[0] + (1 if dx > 0 else -1), c[1]) if abs(dx) >= abs(dy) else \
            (c[0], c[1] + (1 if dy > 0 else -1))
        return g if g in self.rm.free else None

    def _claim_ghost(self, name, point, clear=0.55):
        """Reserve the overhang cell before leaving the cell centre for
        `point`. Needs no other robot on/claiming it and none physically within
        `clear` m of `point` (chassis is 0.48 m wide). Parked robots in the way
        are sent to a refuge. Returns True once reserved (or not needed)."""
        g = self._ghost_cell(name, point)
        if g is None or self.tm.ghosts.get(g) == name:
            return True
        me = self.tm.agents[name]
        blockers = [a for a in self.tm.agents.values() if a is not me and (
            a.cell == g or math.dist(self.poses[a.name][:2], point[:2]) < clear)]
        if not blockers and g not in self.tm.ghosts:
            self.tm.ghosts[g] = name
            return True
        avoid = {g, me.cell, *self.rm.neighbors(g), *self.rm.neighbors(me.cell)}
        for o in blockers:
            if self.phase[o.name][0] == "idle" and not o.path:
                route = self.tm._refuge_route(o, avoid)
                if route:
                    o.path, o.goal = route, route[-1]
                    self._log(f"{o.name} steps aside for {name}")
        return False

    def _park(self, name):
        """Send a robot that just went idle to the nearest cell off the shelf
        fronts and off other robots' paths/goals."""
        a = self.tm.agents[name]
        avoid = set(self.shelf_front)
        for o in self.tm.agents.values():
            if o is not a:
                avoid |= {o.cell, *o.path, *([o.goal] if o.goal else [])}
        if a.cell not in avoid:
            return
        route = self.tm._refuge_route(a, avoid)
        if route:
            a.path, a.goal = route, route[-1]
            self._log(f"{name} parks at {route[-1]}")

    def _stall_check(self, limit=30.0, moved=0.05):
        """A busy robot that has not moved `moved` m in `limit` s while
        travelling asks idle robots within 1.2 m to step aside."""
        for n, (ph, _, _) in self.phase.items():
            t0, p0 = self.progress[n]
            if ph == "idle" or math.dist(p0, self.poses[n][:2]) > moved:
                self.progress[n] = (self.t, tuple(self.poses[n][:2]))
                continue
            if ph in ("to_pick", "to_drop") and self.t - t0 > limit:
                self.stalls += 1
                self.progress[n] = (self.t, tuple(self.poses[n][:2]))
                me = self.tm.agents[n]
                avoid = {me.cell, *me.path, *self.rm.neighbors(me.cell)}
                for o in self.tm.agents.values():
                    if o is me or self.phase[o.name][0] != "idle" or o.path:
                        continue
                    if math.dist(self.poses[o.name][:2], self.poses[n][:2]) < 1.2:
                        route = self.tm._refuge_route(o, avoid | self.shelf_front)
                        if route:
                            o.path, o.goal = route, route[-1]
                            self._log(f"{n} stalled; {o.name} steps aside")

    # ---- shelf lock (Day 27) ---------------------------------------------
    def _leg_shelf(self, tid, pick):
        return self.fleet.tasks[tid].plan["src_shelf" if pick else "dst_shelf"]

    def _begin_leg(self, n, tid, pick):
        """Start the to_pick/to_drop leg if the shelf lock can be taken;
        otherwise wait (sub=1) off the shelf fronts."""
        ph = "to_pick" if pick else "to_drop"
        shelf = self._leg_shelf(tid, pick)
        a = self.tm.agents[n]
        a.hold_for = set()
        if self.lock.get(shelf, n) == n:
            if shelf not in self.lock:
                self._log(f"{n} locks {shelf}")
            self.lock[shelf] = n
            if self.leaving.get(n) == shelf:
                del self.leaving[n]
            self._set(n, ph, tid, 0)
            task = self.fleet.tasks[tid]
            self.tm.set_goal(n, self._cell(task.pick_pose if pick else task.drop_pose))
            return True
        if self.phase[n][:3] != (ph, tid, 1):
            self.lock_waits += 1
            self._log(f"{n} waits for {shelf} (held by {self.lock[shelf]})")
        self._set(n, ph, tid, 1)
        a.goal = None
        self._park(n)
        return False

    def _locks(self):
        """Release locks of robots that have left a shelf front; start legs
        of robots waiting for a lock."""
        for n, shelf in list(self.leaving.items()):
            a = self.tm.agents[n]
            # physical cell, not "at target": Day 27 Isaac run 2 froze because a
            # parked robot never settled within tol of its cell centre
            if a.cell not in self.shelf_cells[shelf] \
                    and self._cell(self.poses[n]) not in self.shelf_cells[shelf]:
                del self.leaving[n]
                if self.lock.get(shelf) == n:
                    del self.lock[shelf]
                    self._log(f"{n} releases {shelf}")
        for n, (ph, tid, sub) in self.phase.items():
            if ph in ("to_pick", "to_drop") and sub == 1 and not self.tm.agents[n].path:
                self._begin_leg(n, tid, ph == "to_pick")

    def _release(self, n, shelf):
        """Robot finished at `shelf`: keep the lock until it is off the front."""
        if self.lock.get(shelf) == n:
            self.leaving[n] = shelf

    def _set(self, name, ph, tid=-1, sub=0):
        self.phase[name] = (ph, tid, sub)
        if ph in TRAFFIC:
            self.tm.pinned.discard(name)   # ghost is released once back at the cell centre
        else:
            self.tm.pinned.add(name)

    # ---- gripper actions ------------------------------------------------
    def action_done(self, name, ok):
        ph, tid, _ = self.phase[name]
        task = self.fleet.tasks[tid]
        if not ok:
            self._log(f"{name} {ph} of {task.box_id} FAILED")
            self.fleet.finish(tid, ok=False, reason=f"{ph} failed", now=self.t)
            self._set(name, "dock_out", tid, 1 if ph == "pick" else 2)
            self.failed_out.add(name)
            return
        self._log(f"{name} {'picked' if ph == 'pick' else 'dropped'} {task.box_id}")
        self._set(name, "dock_out", tid, 1 if ph == "pick" else 2)

    # ---- main step --------------------------------------------------------
    def step(self, poses, dt):
        """poses: name -> (x, y, yaw) map frame; dt: seconds since last step.
        Returns name -> (v, w)."""
        self.poses.update(poses)
        self.t += dt
        for n, (t0, p0) in self.still.items():
            p = self.poses[n]
            dyaw = abs(math.remainder(p[2] - p0[2], math.tau))
            if math.dist(p0[:2], p[:2]) > self.settle_moved or dyaw > 0.05:
                self.still[n] = (self.t, tuple(p))
        self._dt = dt
        self._try_submit()
        for task, robot in self.fleet.dispatch(now=self.t):
            self._log(f"task {task.id} ({task.box_id}) -> {robot.name}")
            self._begin_leg(robot.name, task.id, pick=True)
        self._locks()
        self._separation()
        self._stall_check()
        cmds = {}
        for n in self.phase:
            cmds[n] = self._manip(n) if self.phase[n][0] not in TRAFFIC else None
        tcmds, arrived = self._traffic_cmds()
        moved_tick = False
        if arrived and not self.tm.all_done():
            cells = self.tm.tick()
            moved_tick = True
            self.target = {n: self.rm.to_world(c) for n, c in cells.items()}
            tcmds, arrived = self._traffic_cmds()
        for n, c in tcmds.items():
            cmds[n] = c
        self._account(dt, moved_tick)
        self._arrivals()
        return cmds

    def _traffic_cmds(self):
        cmds, arrived = {}, True
        for n, (ph, _, _) in self.phase.items():
            if ph not in TRAFFIC:
                continue
            # settle on the exact centre (0.04 m) but tick once within tol
            v, w, _ = cmd_to(self.poses[n], self.target[n], tol=0.04)
            cmds[n] = (v, w)
            arrived &= self._tick_ready(n)
        self._arrived = arrived
        return cmds, arrived

    def _at_target(self, n):
        p, t = self.poses[n], self.target[n]
        return math.hypot(p[0] - t[0], p[1] - t[1]) < self.tol

    def _tick_ready(self, n):
        """Lockstep gate only (not used for dock_in)."""
        if self._at_target(n):
            return True
        p, t = self.poses[n], self.target[n]
        d = math.hypot(p[0] - t[0], p[1] - t[1])
        # watchdog: still for settle_s and physically inside the target cell
        if self.t - self.still[n][0] >= self.settle_s \
                and self.rm.to_cell(*p[:2]) == self.rm.to_cell(*t[:2]):
            if self.phase[n][0] in TRAFFIC and n not in self._settled_logged:
                self._settled_logged.add(n)
                self.settled += 1
                self._log(f"{n} settled {d:.2f} m off its cell centre; counts as arrived")
            return True
        self._settled_logged.discard(n)
        return False

    def _arrivals(self):
        for n, (ph, tid, _) in list(self.phase.items()):
            a = self.tm.agents[n]
            if ph in TRAFFIC and self._at_target(n):
                for g in [g for g, o in self.tm.ghosts.items() if o == n]:
                    del self.tm.ghosts[g]
            if ph in ("to_pick", "to_drop") and self.phase[n][2] == 0 and a.done \
                    and not a.hold_for and self._at_target(n):
                self._set(n, "dock_in", tid, 0 if ph == "to_pick" else 10)
            elif ph == "idle":
                self.fleet.robots[n].pose = (*self.poses[n][:2], 0.0)

    def _account(self, dt, ticked):
        for n, (ph, _, _) in self.phase.items():
            rt = self.robot_time[n]
            if ph == "idle":
                rt["idle"] += dt
            else:
                rt["busy"] += dt
                a = self.tm.agents[n]
                if ph in ("to_pick", "to_drop") and (a.waited > 0 or a.hold_for):
                    rt["traffic_wait"] += dt

    def _slot(self, tid, pick):
        t = self.fleet.tasks[tid]
        return t.plan["src"] if pick else t.plan["dst"]

    def _manip(self, n):
        """Docking state machine. dock_in step = sub % 10 (+10 on the drop side):
        0 approach point, 1 face shelf, 5 settle + lidar measure, 6 back up if
        lateral error, 2 creep to the dock, 3 final yaw -> gripper action.
        10..13 drop-side; dock_out sub 1 = after pick, 2 = after drop."""
        ph, tid, sub = self.phase[n]
        p = self.poses[n]
        if ph in ("pick", "drop"):
            return (0.0, 0.0)
        if ph == "dock_in":
            pick = sub < 10
            slot = self._slot(tid, pick)
            ap, dk = slot["approach"], slot["dock"]
            goal = self.dock_goal.get(n) or (dk[0], dk[1], math.radians(dk[2]))
            yaw = goal[2]
            step = sub % 10
            if step == 0 and not self._claim_ghost(n, ap):
                return (0.0, 0.0)                 # wait until the overhang cell is free
            if step == 0:                         # to the approach point
                self.dock_goal.pop(n, None)
                v, w, ok = cmd_to(p, ap, v_max=0.2, tol=0.04)
                if ok:
                    self._set(n, ph, tid, sub + 1)
                return (v, w)
            if step in (1, 3):                    # face the shelf / the measured dock yaw
                err = wrap(yaw - p[2])
                if abs(err) < self.yaw_tol:
                    if step == 3 and not pick and self._place_fix(n, tid, slot, goal, sub):
                        return (0.0, 0.0)
                    if step == 3:
                        self.fixed.pop(n, None)
                        self.box_off.pop(n, None)
                        kind = "pick" if pick else "drop"
                        self._set(n, kind, tid)
                        self.requests.append((n, kind, self.fleet.tasks[tid].box_id))
                    else:
                        self.settle[n] = self.settle_s
                        self._set(n, ph, tid, sub + 4)
                    return (0.0, 0.0)
                return (0.0, max(-0.6, min(0.6, 1.5 * err + math.copysign(0.05, err))))
            if step == 5:                         # stopped: measure the rack face
                self.settle[n] -= self._dt
                if self.settle[n] > 0:
                    return (0.0, 0.0)
                task = self.fleet.tasks[tid]
                shelf = task.plan["src_shelf" if pick else "dst_shelf"]
                rel = self.measure_fn(n, self.fleet.registry.shelves[shelf], slot) \
                    if self.measure_fn else None
                if rel is None:
                    self._log(f"{n} rack not measured; AMCL dock")
                    self._set(n, ph, tid, sub - 3)
                    return (0.0, 0.0)
                goal = compose(p, rel)
                off = self.box_off.get(n, (0.0, 0.0))
                self.dock_goal[n] = goal = (goal[0] - off[0], goal[1] - off[1], goal[2])
                lat = -math.sin(goal[2]) * (p[0] - goal[0]) + math.cos(goal[2]) * (p[1] - goal[1])
                self._log(f"{n} lidar dock: lateral {lat:+.3f} m")
                self._set(n, ph, tid, sub + 1 if abs(lat) > self.backup_lat else sub - 3)
                return (0.0, 0.0)
            if step == 6:                         # back up along the corrected line
                back = (goal[0] - self.backup_m * math.cos(yaw), goal[1] - self.backup_m * math.sin(yaw))
                v, w, ok = dock_cmd(p, back, yaw, v_max=0.12, tol=0.02)
                if ok:
                    if n in self.remeasure and self.measure_fn:   # re-dock: face + lidar again
                        self.remeasure.discard(n)
                        self._set(n, ph, tid, sub - 5)
                    else:
                        self.remeasure.discard(n)
                        self._set(n, ph, tid, sub - 4)
                return (v, w)
            if step == 2:                         # creep in
                v, w, ok = dock_cmd(p, goal, yaw, v_max=0.12)
                if ok:
                    self._set(n, ph, tid, sub + 1)
                return (v, w)
        if ph == "dock_out":
            pick = sub == 1
            slot = self._slot(tid, pick)
            ap = slot["approach"]
            g = self.dock_goal.get(n)
            v, w, ok = dock_cmd(p, ap, g[2] if g else math.radians(slot["dock"][2]), v_max=0.2, tol=0.03)
            if ok:
                task = self.fleet.tasks[tid]
                self._release(n, self._leg_shelf(tid, pick))
                if n in self.failed_out:          # task already finished as failed
                    self.failed_out.discard(n)
                    self._set(n, "idle")
                    self.tm.agents[n].goal = None
                elif pick:
                    self._begin_leg(n, tid, pick=False)
                else:
                    self.fleet.finish(tid, ok=True, pose=(*p[:2], 0.0), now=self.t)
                    self._log(f"{n} done task {tid} ({task.box_id})")
                    self._set(n, "idle")
                    self.tm.agents[n].goal = None
                    self._park(n)
                return (0.0, 0.0)
            return (v, w)
        return (0.0, 0.0)

    def _place_fix(self, n, tid, slot, goal, sub, tol=0.05, max_fix=2):
        """Before a drop: the virtual gripper keeps the box's pick-time offset,
        so check where the held box really is (place_error_fn, e.g. /boxes)
        and re-dock (up to max_fix times, Day 26: one re-dock could stop short)
        with the dock goal shifted by the error. Returns True if a re-dock was
        started."""
        if self.place_error_fn is None or self.fixed.get(n, 0) >= max_fix:
            return False
        err = self.place_error_fn(n, self.fleet.tasks[tid].box_id, slot)
        if err is None or math.hypot(*err) < tol:
            return False
        self.fixed[n] = self.fixed.get(n, 0) + 1
        o = self.box_off.get(n, (0.0, 0.0))
        self.box_off[n] = (o[0] + err[0], o[1] + err[1])
        self.remeasure.add(n)
        self.dock_goal[n] = (goal[0] - err[0], goal[1] - err[1], goal[2])
        self._log(f"{n} held box off by {err[0]:+.3f},{err[1]:+.3f} m; re-docking")
        self._set(n, "dock_in", tid, sub + 3)     # step 6: back up, then creep in again
        return True

    def _separation(self):
        names = sorted(self.poses)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                d = math.dist(self.poses[a][:2], self.poses[b][:2])
                self.min_sep = min(self.min_sep, d)
                if d < 0.3:
                    self.collisions += 1

    def report(self):
        tasks = list(self.fleet.tasks.values())
        return {
            "sim_time_s": round(self.t, 2),
            "tasks_done": sum(t.state == "done" for t in tasks),
            "tasks_failed": sum(t.state == "failed" for t in tasks),
            "rejected": len(self.rejected),
            "min_separation_m": None if math.isinf(self.min_sep) else round(self.min_sep, 3),
            "deadlocks_resolved": self.tm.deadlocks,
            "stalls": self.stalls,
            "settled": self.settled,
            "lock_waits": self.lock_waits,
            "per_robot_tasks": {n: sum(t.robot == n for t in tasks) for n in self.fleet.robots},
            "tasks": [{"id": t.id, "box": t.box_id, "robot": t.robot, "state": t.state,
                       "dst": [t.plan["dst_shelf"], t.plan["dst_slot"]],
                       "t_assign": round(t.t_assign, 2), "t_done": round(t.t_done, 2)} for t in tasks],
            "metrics": compute_metrics(tasks, self.robot_time, self.t) if self.t > 0 else {},
            "boxes": {b: list(v) for b, v in sorted(self.fleet.registry.boxes.items())},
        }
