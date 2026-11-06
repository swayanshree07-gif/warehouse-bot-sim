"""Mock robot backend for the MoveBox state machine (Day 19).

Simulates the robot pose, lift height, held box and box positions at the level
of the backend interface in mission.py, with no physics. Used by the unit tests
and by `task_manager --ros-args -p backend:=mock` (no sim needed).

  fail_at   {method_name: message} - that call fails (once per entry use
            `fail_once`), e.g. {"navigate": "goal unreachable"}
  step_time seconds each call sleeps (to make feedback/cancel observable)
  missing   box ids that are not where the registry says (moved to the floor
            far away), like the sim's /sim/remove_box (Day 22)
"""
import math
import time


class MockRobot:
    def __init__(self, registry, step_time=0.0, fail_at=None, fail_once=False, grip_tol=0.10,
                 missing=()):
        self.reg = registry
        self.step_time = step_time
        self.fail_at = dict(fail_at or {})
        self.fail_once = fail_once
        self.grip_tol = grip_tol
        self.pose = [0.0, 0.0, 0.0]
        self.height = 0.0
        self.held = ""
        self.calls = []   # (method, arg) log, for tests
        # ground-truth box centres in the map frame
        self.box_pos = {}
        for b, (shelf, idx) in registry.boxes.items():
            self.box_pos[b] = tuple(registry.slot(shelf, idx)["center"])
        for i, b in enumerate(missing):
            self.box_pos[b] = (6.6, -3.6 + 0.3 * i, 0.1)

    def _step(self, name, arg=None):
        self.calls.append((name, arg))
        if self.step_time:
            time.sleep(self.step_time)
        if name in self.fail_at:
            msg = self.fail_at[name]
            if self.fail_once:
                del self.fail_at[name]
            return False, msg
        return True, ""

    def _goto(self, pose):
        self.pose = [float(v) for v in pose]
        if self.held:
            yaw = math.radians(self.pose[2])
            r = self.reg.data.get("gripper", {}).get("reach", 0.5)
            z = self.box_pos[self.held][2]
            self.box_pos[self.held] = (self.pose[0] + r * math.cos(yaw),
                                       self.pose[1] + r * math.sin(yaw), z)

    def navigate(self, pose):
        ok, msg = self._step("navigate", tuple(pose))
        if ok:
            self._goto(pose)
        return ok, msg

    def dock(self, pose):
        ok, msg = self._step("dock", tuple(pose))
        if ok:
            self._goto(pose)
        return ok, msg

    def undock(self, pose):
        ok, msg = self._step("undock", tuple(pose))
        if ok:
            self._goto(pose)
        return ok, msg

    def lift(self, height):
        ok, msg = self._step("lift", height)
        if ok:
            if self.held:
                b = self.box_pos[self.held]
                self.box_pos[self.held] = (b[0], b[1], b[2] + height - self.height)
            self.height = height
        return ok, msg

    def grip_point(self):
        yaw = math.radians(self.pose[2])
        r = self.reg.data.get("gripper", {}).get("reach", 0.5)
        return self.pose[0] + r * math.cos(yaw), self.pose[1] + r * math.sin(yaw)

    def attach(self):
        ok, msg = self._step("attach")
        if not ok:
            return ok, msg
        if self.held:
            return False, f"already holding {self.held}"
        gx, gy = self.grip_point()
        best = min(self.box_pos, key=lambda b: math.hypot(self.box_pos[b][0] - gx, self.box_pos[b][1] - gy))
        d = math.hypot(self.box_pos[best][0] - gx, self.box_pos[best][1] - gy)
        if d > self.grip_tol:
            return False, f"no box within {self.grip_tol} m of grip point"
        self.held = best
        return True, best

    def detach(self):
        ok, msg = self._step("detach")
        if not ok:
            return ok, msg
        if not self.held:
            return False, "not holding a box"
        b, self.held = self.held, ""
        return True, b

    def stop(self):
        self.calls.append(("stop", None))

    def box_position(self, box_id):
        return self.box_pos.get(box_id)

    def box_states(self):
        """Ground truth {box_id: (shelf, slot, held)} like RosRobot's /boxes."""
        out = {}
        for b, p in self.box_pos.items():
            shelf, slot = (None, -1) if b == self.held else self.reg.slot_at(p)
            out[b] = (shelf or "", slot, b == self.held)
        return out
