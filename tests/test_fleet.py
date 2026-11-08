"""Day 15: fleet core - nearest free robot assignment, queueing, claims."""
import os
import sys

import pytest

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(ROOT, "ros_ws", "src", "warehouse_tasks"))
from warehouse_tasks.fleet import (ASSIGNED, BUSY, DEFAULT_NAMESPACES, DONE,  # noqa: E402
                                   FAILED, IDLE, QUEUED, FleetManager)
from warehouse_tasks.registry import Registry, TaskError  # noqa: E402

REG = os.path.join(ROOT, "ros_ws", "src", "warehouse_bringup", "config", "registry.yaml")


@pytest.fixture
def fm():
    f = FleetManager(Registry.load(REG))
    # robot1 near shelf_A, robot2 near shelf_B, robot3 near shelf_D
    f.add_robot("robot1", (-0.5, 0.0, 0))
    f.add_robot("robot2", (4.5, 0.5, 0))
    f.add_robot("robot3", (4.5, -0.5, 0))
    return f


def test_three_namespaced_robots():
    assert len(DEFAULT_NAMESPACES) >= 3 and len(set(DEFAULT_NAMESPACES)) == len(DEFAULT_NAMESPACES)


def test_nearest_free_robot_wins(fm):
    t = fm.submit("box_04", "shelf_D")          # pick at shelf_B -> robot2
    (task, robot), = fm.dispatch()
    assert task is t and robot.name == "robot2" and t.state == ASSIGNED
    t2 = fm.submit("box_00", "shelf_D")         # pick at shelf_A -> robot1
    assert fm.dispatch()[0][1].name == "robot1"
    assert t2.robot == "robot1"


def test_busy_robot_skipped(fm):
    fm.submit("box_03", "shelf_D")
    fm.submit("box_05", "shelf_D")              # also at shelf_B, robot2 busy
    got = [r.name for _, r in fm.dispatch()]
    assert got[0] == "robot2" and got[1] == "robot3"   # next nearest


def test_tasks_wait_when_all_busy_then_fifo():
    # The real registry has only 2 free slots; drop two boxes so 4 tasks fit.
    reg = Registry.load(REG)
    del reg.boxes["box_05"], reg.boxes["box_08"]
    fm = FleetManager(reg)
    for i, n in enumerate(("robot1", "robot2", "robot3")):
        fm.add_robot(n, (i, 0.0, 0))
    ids = [fm.submit(b, "shelf_D").id for b in ("box_00", "box_01")]
    ids.append(fm.submit("box_02", "shelf_B").id)
    t4 = fm.submit("box_06", "shelf_C")
    assert len(fm.dispatch()) == 3
    assert fm.queue == [t4.id] and t4.state == QUEUED
    assert fm.dispatch() == []
    first = fm.tasks[ids[0]]
    fm.finish(first.id, ok=True, pose=first.drop_pose)
    (task, robot), = fm.dispatch()
    assert task is t4 and robot.name == first.robot


def test_double_booking_rejected(fm):
    fm.submit("box_00", "shelf_D", 1)
    with pytest.raises(TaskError, match="active task"):
        fm.submit("box_00", "shelf_D", 2)
    with pytest.raises(TaskError, match="claimed"):
        fm.submit("box_01", "shelf_D", 1)
    t = fm.submit("box_02", "shelf_D")           # auto slot skips claimed 1
    assert t.dst_key == ("shelf_D", 2)
    with pytest.raises(TaskError, match="no free unclaimed"):
        fm.submit("box_03", "shelf_D")


def test_registry_errors_propagate(fm):
    with pytest.raises(TaskError, match="unknown box"):
        fm.submit("box_99", "shelf_D")
    with pytest.raises(TaskError, match="occupied"):
        fm.submit("box_00", "shelf_B", 0)


def test_finish_updates_belief_and_frees_robot(fm):
    t = fm.submit("box_00", "shelf_D", 2)
    fm.dispatch()
    fm.finish(t.id, ok=True, pose=t.drop_pose, now=10.0)
    assert t.state == DONE and fm.registry.boxes["box_00"] == ("shelf_D", 2)
    assert fm.robots[t.robot].state == IDLE and fm.robots[t.robot].pose == t.drop_pose
    with pytest.raises(ValueError):
        fm.finish(t.id, ok=True)
    t2 = fm.submit("box_01", "shelf_D")
    fm.dispatch()
    fm.finish(t2.id, ok=False, reason="nav failed")
    assert t2.state == FAILED and fm.registry.boxes["box_01"] == ("shelf_A", 1)
    assert fm.status()["done"] == 1 and fm.status()["failed"] == 1


def test_offline_robot_not_assigned(fm):
    fm.set_offline("robot2")
    fm.submit("box_04", "shelf_D")
    assert fm.dispatch()[0][1].name == "robot3"
    assert fm.robots["robot3"].state == BUSY
    with pytest.raises(ValueError):
        fm.set_offline("robot3")


def test_custom_distance_fn(fm):
    # e.g. a roadmap path length: pretend robots at x < 0 are "closer"
    fm.distance_fn = lambda a, b: 0 if a[0] < 0 else 5
    fm.submit("box_04", "shelf_D")
    assert fm.dispatch()[0][1].name == "robot1"
