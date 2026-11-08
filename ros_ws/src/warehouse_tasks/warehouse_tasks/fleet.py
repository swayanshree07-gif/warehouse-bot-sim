"""Fleet core (Day 15): robots, pick/drop tasks and nearest-free-robot assignment.

Pure Python (no ROS) so it is unit tested and shared by the light simulator
(Day 17) and the ROS 2 fleet_manager node (Day 23).

Rules:
- Tasks are queued FIFO. `dispatch()` hands out tasks in queue order; each
  goes to the free robot nearest (Euclidean, or a supplied `distance_fn`
  such as a roadmap path length) to the task's pick approach pose. Ties are
  broken by robot name, so results are deterministic.
- When no robot is free the task waits in the queue.
- A task is validated against the registry at submit time AND against the
  tasks already in flight: the same box, or the same target slot, cannot be
  used by two active tasks (the registry belief only changes when a task
  finishes, so this "claim" check prevents double booking).
"""
import math
from dataclasses import dataclass, field

from .registry import TaskError

DEFAULT_NAMESPACES = ("robot1", "robot2", "robot3")

IDLE, BUSY, OFFLINE = "idle", "busy", "offline"
QUEUED, ASSIGNED, DONE, FAILED = "queued", "assigned", "done", "failed"


@dataclass
class Robot:
    name: str                      # also the ROS namespace, e.g. "robot1"
    pose: tuple                    # (x, y, yaw_deg) in the map frame
    state: str = IDLE
    task_id: int = -1


@dataclass
class Task:
    id: int
    box_id: str
    plan: dict                     # Registry.plan() output
    state: str = QUEUED
    robot: str = ""
    reason: str = ""
    t_submit: float = 0.0
    t_assign: float = -1.0
    t_done: float = -1.0

    @property
    def pick_pose(self):
        return tuple(self.plan["src"]["approach"])

    @property
    def drop_pose(self):
        return tuple(self.plan["dst"]["approach"])

    @property
    def dst_key(self):
        return (self.plan["dst_shelf"], self.plan["dst_slot"])


def euclid(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


@dataclass
class FleetManager:
    registry: object
    robots: dict = field(default_factory=dict)        # name -> Robot
    distance_fn: object = euclid
    tasks: dict = field(default_factory=dict)          # id -> Task
    queue: list = field(default_factory=list)          # task ids, FIFO
    _next_id: int = 1

    def add_robot(self, name, pose):
        if name in self.robots:
            raise ValueError(f"duplicate robot {name}")
        self.robots[name] = Robot(name, tuple(pose))
        return self.robots[name]

    # --- tasks ---------------------------------------------------------------
    def _active(self):
        return [t for t in self.tasks.values() if t.state in (QUEUED, ASSIGNED)]

    def submit(self, box_id, target_shelf, target_slot=-1, now=0.0):
        """Validate and queue a pick/drop task. Raises TaskError on conflict."""
        active = self._active()
        if any(t.box_id == box_id for t in active):
            raise TaskError(f"{box_id} already has an active task")
        # Slots claimed by active tasks count as occupied; if target_slot is
        # auto (-1) pick the first slot that is free AND unclaimed.
        claimed = {t.dst_key for t in active}
        # Boxes being moved away free their source slot only when done.
        if target_slot < 0 and target_shelf in self.registry.shelves:
            free = [s for s in self.registry.free_slots(target_shelf)
                    if (target_shelf, s) not in claimed]
            if not free:
                raise TaskError(f"no free unclaimed slot on {target_shelf}")
            target_slot = free[0]
        if (target_shelf, target_slot) in claimed:
            raise TaskError(f"slot claimed by another task: {target_shelf} slot {target_slot}")
        plan = self.registry.plan(box_id, target_shelf, target_slot)
        t = Task(self._next_id, box_id, plan, t_submit=now)
        self._next_id += 1
        self.tasks[t.id] = t
        self.queue.append(t.id)
        return t

    def free_robots(self):
        return [r for r in self.robots.values() if r.state == IDLE]

    def nearest_free(self, pose):
        free = self.free_robots()
        if not free:
            return None
        return min(free, key=lambda r: (self.distance_fn(r.pose, pose), r.name))

    def dispatch(self, now=0.0):
        """Assign queued tasks (FIFO) to the nearest free robots.
        Returns [(task, robot), ...] assigned in this call."""
        out = []
        while self.queue:
            r = self.nearest_free(self.tasks[self.queue[0]].pick_pose)
            if r is None:
                break
            t = self.tasks[self.queue.pop(0)]
            t.state, t.robot, t.t_assign = ASSIGNED, r.name, now
            r.state, r.task_id = BUSY, t.id
            out.append((t, r))
        return out

    def finish(self, task_id, ok, reason="", pose=None, now=0.0):
        """Mark a task done/failed and free its robot. On success the registry
        belief moves the box to the target slot."""
        t = self.tasks[task_id]
        if t.state != ASSIGNED:
            raise ValueError(f"task {task_id} is {t.state}, not assigned")
        r = self.robots[t.robot]
        t.state, t.reason, t.t_done = (DONE if ok else FAILED), reason, now
        if ok:
            self.registry.move(t.box_id, t.plan["dst_shelf"], t.plan["dst_slot"])
        r.state, r.task_id = IDLE, -1
        if pose is not None:
            r.pose = tuple(pose)
        return t

    def set_offline(self, name, offline=True):
        r = self.robots[name]
        if offline and r.state == BUSY:
            raise ValueError(f"{name} is busy")
        r.state = OFFLINE if offline else IDLE

    def status(self):
        return {
            "robots": {n: (r.state, r.task_id) for n, r in self.robots.items()},
            "queued": list(self.queue),
            "done": sum(t.state == DONE for t in self.tasks.values()),
            "failed": sum(t.state == FAILED for t in self.tasks.values()),
        }
