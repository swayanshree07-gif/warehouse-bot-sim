"""MoveBox pick/place state machine (Day 19). Pure Python, no ROS imports.

The state machine drives a *robot backend* with this interface (every method
returns (ok: bool, message: str) and blocks until the step is done):

  navigate(pose)    Nav2 to [x, y, yaw_deg] in the map frame (the approach pose)
  dock(pose)        precise straight-line final approach to the dock pose
  undock(pose)      reverse straight back out to the approach pose
  lift(height)      lift carriage to height (m), wait until reached
  attach()          gripper attach; message = id of the box now held
  detach()          gripper release; message = id of the released box
  stop()            zero velocity (best effort, no return value needed)
  box_position(id)  optional: ground-truth box centre (x, y, z) in the map
                    frame or None (used for the result's place_error)

Sequence (state names are the MoveBox feedback `state`):
  NAV_TO_PICK   navigate(src approach)
  DOCK_PICK     dock(src dock)
  LIFT          attach() (must return the requested box id), lift(clearance)
  UNDOCK_PICK   undock(src approach)
  NAV_TO_PLACE  navigate(dst approach)
  DOCK_PLACE    dock(dst dock)
  LOWER         lift(0), detach()
  RETREAT       undock(dst approach)
On a failed step the robot is stopped and the result names the state and the
backend's message. If it failed at/near a rack without holding a box
(DOCK_PICK, attach in LIFT, DOCK_PLACE) it first backs out to the approach pose
(RECOVER), so the next task's Nav2 goal does not start inside the inflation
zone. With a box held it stays put (backing out could drag the box). If the box was picked up already, the failure message says
so ("box still held") so an operator knows the gripper state. Cancellation is
checked between steps.
"""
import math

STATES = ["NAV_TO_PICK", "DOCK_PICK", "LIFT", "UNDOCK_PICK",
          "NAV_TO_PLACE", "DOCK_PLACE", "LOWER", "RETREAT"]


# failed state -> which side's approach pose to back out to (only if no box is held)
RECOVER = {"DOCK_PICK": "src", "LIFT": "src", "DOCK_PLACE": "dst"}


def _safe(fn):
    try:
        return fn()
    except Exception as e:   # backend bug or ROS error: fail the task, not the node
        return False, f"{type(e).__name__}: {e}"


class Outcome:
    def __init__(self, success, message, shelf="", slot=-1, place_error=-1.0,
                 canceled=False, state=""):
        self.success = success
        self.message = message
        self.shelf = shelf
        self.slot = slot
        self.place_error = place_error
        self.canceled = canceled
        self.state = state   # state in which the task ended

    def __repr__(self):
        return (f"Outcome(success={self.success}, message={self.message!r}, "
                f"shelf={self.shelf!r}, slot={self.slot}, state={self.state!r})")


def run_move_box(robot, plan, lift_clearance, feedback=None, is_canceled=None):
    """Execute a validated plan (Registry.plan) on `robot`. Returns an Outcome.
    feedback(state, progress) is called on entering each state."""
    feedback = feedback or (lambda s, p: None)
    is_canceled = is_canceled or (lambda: False)
    box = plan["box_id"]
    src, dst = plan["src"], plan["dst"]
    held = False

    def attach():
        ok, msg = robot.attach()
        if ok and msg != box:
            # picked the wrong box: put it back down and give up
            robot.detach()
            return False, f"attached {msg} instead of {box}"
        return ok, msg

    def lower_and_detach():
        ok, msg = robot.lift(0.0)
        if not ok:
            return ok, msg
        return robot.detach()

    steps = [
        ("NAV_TO_PICK", lambda: robot.navigate(src["approach"])),
        ("DOCK_PICK", lambda: robot.dock(src["dock"])),
        ("LIFT", attach),
        ("LIFT", lambda: robot.lift(lift_clearance)),
        ("UNDOCK_PICK", lambda: robot.undock(src["approach"])),
        ("NAV_TO_PLACE", lambda: robot.navigate(dst["approach"])),
        ("DOCK_PLACE", lambda: robot.dock(dst["dock"])),
        ("LOWER", lower_and_detach),
        ("RETREAT", lambda: robot.undock(dst["approach"])),
    ]
    last = None
    for i, (state, fn) in enumerate(steps):
        if is_canceled():
            robot.stop()
            return Outcome(False, f"canceled before {state}" + (" (box still held)" if held else ""),
                           canceled=True, state=state)
        if state != last:
            feedback(state, STATES.index(state) / len(STATES))
            last = state
        ok, msg = _safe(fn)
        if not ok:
            robot.stop()
            note = " (box still held)" if held else ""
            back = RECOVER.get(state)
            if back and not held:
                # docked (or part-way in) without a box: back out to the approach
                # pose so the robot is not left inside the rack's inflation zone
                rok, rmsg = _safe(lambda: robot.undock(plan[back]["approach"]))
                robot.stop()
                note += "; backed out to approach" if rok else f"; back-out failed: {rmsg}"
            return Outcome(False, f"{state} failed: {msg}{note}", state=state)
        if fn is attach:
            held = True
        if state == "LOWER":
            held = False
    feedback("DONE", 1.0)
    err = -1.0
    pos = robot.box_position(box) if hasattr(robot, "box_position") else None
    if pos is not None:
        c = dst["center"]
        err = math.hypot(pos[0] - c[0], pos[1] - c[1])
    return Outcome(True, f"moved {box} to {plan['dst_shelf']} slot {plan['dst_slot']}",
                   shelf=plan["dst_shelf"], slot=plan["dst_slot"], place_error=err, state="DONE")
