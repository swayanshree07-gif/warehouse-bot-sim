"""Day 21: traffic-manager path follower for the fleet (pure Python, no ROS).

Decision (Day 21): instead of 3 Nav2 stacks (~9 lifecycle nodes + 3 costmap
pairs per robot), the fleet is driven by the Day 16 TrafficManager: it plans and
reserves cells, and a tiny per-robot controller turns "go to the centre of this
cell" into (v, w) commands. Ticks are synchronous: the manager only advances
once every robot that was told to move has reached its cell centre, so the
cell reservation guarantees still hold in the continuous world.
"""
import math

from .traffic import TrafficManager


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def cmd_to(pose, target, v_max=0.4, w_max=1.0, tol=0.08):
    """(v, w, arrived) to drive a diff-drive robot at pose=(x, y, yaw) to target=(x, y)."""
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    d = math.hypot(dx, dy)
    if d < tol:
        return 0.0, 0.0, True
    err = wrap(math.atan2(dy, dx) - pose[2])
    w = max(-w_max, min(w_max, 2.0 * err))
    if abs(err) > 0.35:  # turn in place first
        return 0.0, w, False
    v = min(v_max, 0.8 * d) * math.cos(err)
    return max(0.05, v), w, False


class FleetFollower:
    """Couples a TrafficManager with continuous robot poses (map frame)."""

    def __init__(self, roadmap, starts, tol=0.08, **ctl):
        self.rm = roadmap
        self.tm = TrafficManager(roadmap)
        self.tol, self.ctl = tol, ctl
        for name, (x, y) in starts.items():
            self.tm.add_robot(name, roadmap.nearest_free(roadmap.to_cell(x, y)))
        self.target = {n: self.rm.to_world(a.cell) for n, a in self.tm.agents.items()}
        self.ticks = 0
        self.arrived = False

    def set_goal(self, name, xy):
        self.tm.set_goal(name, self.rm.nearest_free(self.rm.to_cell(*xy)))

    def step(self, poses):
        """poses: name -> (x, y, yaw). Returns name -> (v, w); advances the
        traffic manager when every robot sits at its current target cell."""
        cmds, arrived = self._cmds(poses)
        if arrived and not self.tm.all_done():
            cells = self.tm.tick()
            self.ticks += 1
            self.target = {n: self.rm.to_world(c) for n, c in cells.items()}
            cmds, arrived = self._cmds(poses)
        self.arrived = arrived
        return cmds

    def _cmds(self, poses):
        cmds, arrived = {}, True
        for n, p in poses.items():
            v, w, ok = cmd_to(p, self.target[n], tol=self.tol, **self.ctl)
            cmds[n] = (v, w)
            arrived &= ok
        return cmds, arrived

    def done(self):
        """All robots at their goal cells (physically, not just logically)."""
        return self.tm.all_done() and self.arrived
