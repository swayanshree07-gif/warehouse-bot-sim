"""Day 19: MoveBox registry validation + pick/place state machine on a mock robot."""
import os
import sys

import pytest

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(ROOT, "ros_ws", "src", "warehouse_tasks"))
from warehouse_tasks.mission import STATES, run_move_box  # noqa: E402
from warehouse_tasks.mock_robot import MockRobot  # noqa: E402
from warehouse_tasks.registry import Registry, TaskError  # noqa: E402

REG = os.path.join(ROOT, "ros_ws", "src", "warehouse_bringup", "config", "registry.yaml")


@pytest.fixture
def reg():
    return Registry.load(REG)


def run(reg, robot, box, shelf, slot=-1, **kw):
    plan = reg.plan(box, shelf, slot)
    fb = []
    out = run_move_box(robot, plan, reg.lift_clearance, feedback=lambda s, p: fb.append((s, p)), **kw)
    return out, fb


def test_plan_picks_first_free_slot(reg):
    p = reg.plan("box_09", "shelf_D")
    assert (p["src_shelf"], p["src_slot"]) == ("shelf_D", 0)
    assert (p["dst_shelf"], p["dst_slot"]) == ("shelf_D", 1)
    assert p["dst"]["dock"] == [4.5, -1.5, -90.0]


@pytest.mark.parametrize("args,reason", [
    (("box_99", "shelf_D"), "unknown box"),
    (("box_00", "shelf_Z"), "unknown shelf"),
    (("box_00", "shelf_D", 0), "slot occupied"),
    (("box_00", "shelf_D", 7), "unknown slot"),
    (("box_00", "shelf_A", 0), "already at"),
    (("box_09", "shelf_A"), "no free slot"),
])
def test_plan_rejects_invalid(reg, args, reason):
    with pytest.raises(TaskError, match=reason):
        reg.plan(*args)


def test_happy_path_state_order_and_calls(reg):
    robot = MockRobot(reg)
    out, fb = run(reg, robot, "box_02", "shelf_D", 2)
    assert out.success, out
    assert (out.shelf, out.slot) == ("shelf_D", 2)
    assert [s for s, _ in fb] == STATES + ["DONE"]
    progress = [p for _, p in fb]
    assert progress == sorted(progress) and progress[-1] == 1.0
    names = [c[0] for c in robot.calls]
    assert names == ["navigate", "dock", "attach", "lift", "undock",
                     "navigate", "dock", "lift", "detach", "undock"]
    src = reg.slot("shelf_A", 2)
    dst = reg.slot("shelf_D", 2)
    assert robot.calls[0][1] == tuple(src["approach"])
    assert robot.calls[1][1] == tuple(src["dock"])
    assert robot.calls[3][1] == pytest.approx(reg.lift_clearance)
    assert robot.calls[6][1] == tuple(dst["dock"])
    assert robot.calls[7][1] == 0.0
    assert robot.held == "" and robot.height == 0.0
    assert out.place_error == pytest.approx(0.0, abs=1e-6)
    assert robot.box_pos["box_02"][2] == pytest.approx(dst["center"][2])


@pytest.mark.parametrize("method,state", [
    ("navigate", "NAV_TO_PICK"), ("dock", "DOCK_PICK"), ("attach", "LIFT"), ("detach", "LOWER"),
])
def test_step_failure_aborts_with_reason(reg, method, state):
    robot = MockRobot(reg, fail_at={method: "boom"})
    out, fb = run(reg, robot, "box_09", "shelf_D")
    assert not out.success and not out.canceled
    assert out.state == state and out.message.startswith(f"{state} failed: boom")
    assert robot.calls[-1] == ("stop", None)
    assert fb[-1][0] == state


def test_failure_after_pick_reports_box_held(reg):
    robot = MockRobot(reg, fail_at={"navigate": "unreachable"}, fail_once=False)
    # first navigate fails -> nothing held
    out, _ = run(reg, robot, "box_09", "shelf_D")
    assert "still held" not in out.message
    # let the first navigate pass, fail the second one (NAV_TO_PLACE)
    robot = MockRobot(reg)
    orig = robot.navigate
    n = {"i": 0}

    def nav(pose):
        n["i"] += 1
        return (False, "unreachable") if n["i"] == 2 else orig(pose)
    robot.navigate = nav
    out, _ = run(reg, robot, "box_09", "shelf_D")
    assert out.state == "NAV_TO_PLACE" and "box still held" in out.message
    assert robot.held == "box_09"


def test_wrong_box_is_released(reg):
    robot = MockRobot(reg)
    # mis-registered belief: the task manager thinks box_08 is at shelf_D slot 0
    plan = reg.plan("box_09", "shelf_D")
    plan["box_id"] = "box_08"
    out = run_move_box(robot, plan, reg.lift_clearance)
    assert not out.success and "attached box_09 instead of box_08" in out.message
    assert robot.held == ""


def test_attach_out_of_reach_fails(reg):
    robot = MockRobot(reg)
    robot.dock = lambda pose: (True, "")   # "docked" but the robot never moved
    out, _ = run(reg, robot, "box_09", "shelf_D")
    assert out.state == "LIFT" and "no box within" in out.message


def test_cancel_between_steps(reg):
    robot = MockRobot(reg)
    calls = {"n": 0}

    def canceled():
        calls["n"] += 1
        return calls["n"] > 3
    out, _ = run(reg, robot, "box_09", "shelf_D", is_canceled=canceled)
    assert out.canceled and not out.success
    assert out.state == "LIFT" and "box still held" in out.message   # attach done, lift pending


def test_backend_exception_becomes_failure(reg):
    robot = MockRobot(reg)

    def bad(_pose):
        raise RuntimeError("tf lookup failed")
    robot.dock = bad
    out, _ = run(reg, robot, "box_09", "shelf_D")
    assert out.state == "DOCK_PICK" and "RuntimeError: tf lookup failed" in out.message


def test_registry_belief_update_sequence(reg):
    r = reg.copy()
    robot = MockRobot(r)
    for box, shelf, want_slot in [("box_09", "shelf_D", 1), ("box_00", "shelf_D", 0)]:
        plan = r.plan(box, shelf)
        assert plan["dst_slot"] == want_slot
        out = run_move_box(robot, plan, r.lift_clearance)
        assert out.success, out
        r.move(box, out.shelf, out.slot)
    assert r.free_slots("shelf_D") == [2]
    assert r.free_slots("shelf_A") == [0]
    assert reg.free_slots("shelf_D") == [1, 2], "copy() must not share box state"


@pytest.mark.parametrize("method,state,back", [
    ("dock", "DOCK_PICK", "src"), ("attach", "LIFT", "src"),
])
def test_failure_at_rack_backs_out(reg, method, state, back):
    robot = MockRobot(reg, fail_at={method: "boom"})
    out, _ = run(reg, robot, "box_09", "shelf_D")
    assert out.state == state and "backed out to approach" in out.message
    src = reg.slot("shelf_D", 0)
    assert ("undock", tuple(src["approach"])) in robot.calls
    assert robot.pose == [float(v) for v in src["approach"]]


def test_failure_with_box_held_does_not_move(reg):
    robot = MockRobot(reg)
    orig = robot.lift
    robot.lift = lambda h: (False, "stuck") if h == 0.0 else orig(h)
    out, _ = run(reg, robot, "box_09", "shelf_D")
    assert out.state == "LOWER" and "box still held" in out.message
    assert "backed out" not in out.message
    assert [c[0] for c in robot.calls].count("undock") == 1   # only UNDOCK_PICK


def test_nav_failure_no_back_out(reg):
    robot = MockRobot(reg, fail_at={"navigate": "unreachable"})
    out, _ = run(reg, robot, "box_09", "shelf_D")
    assert "backed out" not in out.message
    assert "undock" not in [c[0] for c in robot.calls]
