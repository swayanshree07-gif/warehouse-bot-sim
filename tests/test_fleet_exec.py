"""Day 25: FleetExecutor on kinematic unicycle robots (no ROS, no GPU)."""
import math
import os
import sys

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(ROOT, "ros_ws", "src", "warehouse_tasks"))
from warehouse_tasks.fleet_exec import FleetExecutor, dock_cmd  # noqa: E402
from warehouse_tasks.registry import Registry  # noqa: E402
from warehouse_tasks.traffic import Roadmap  # noqa: E402

BR = os.path.join(ROOT, "ros_ws", "src", "warehouse_bringup")
STARTS = {"robot1": (-1.0, 0.0, 0.0), "robot2": (2.0, 0.0, 0.0), "robot3": (5.0, 0.0, 0.0)}
# Same 6 tasks as scripts/check_fleet_isaac.sh (chained: each pair frees the next slots)
TASKS = [("box_00", "shelf_D"), ("box_04", "shelf_D"), ("box_06", "shelf_A"),
         ("box_07", "shelf_B"), ("box_01", "shelf_C"), ("box_03", "shelf_C")]


def make():
    return FleetExecutor(Roadmap.from_map(os.path.join(BR, "maps", "warehouse.yaml")),
                         Registry.load(os.path.join(BR, "config", "registry.yaml")), STARTS)


def run(ex, poses, max_t=2000.0, dt=0.1, action_s=2.0, fail=()):
    timers = {}
    while ex.busy():
        assert ex.t < max_t, ex.fleet.status()
        cmds = ex.step(poses, dt)
        for n, (v, w) in cmds.items():
            x, y, th = poses[n]
            poses[n] = (x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w * dt)
        while ex.requests:
            n, kind, box = ex.requests.pop(0)
            timers[n] = [action_s, box not in fail]
        for n in list(timers):
            timers[n][0] -= dt
            if timers[n][0] <= 0:
                ex.action_done(n, timers.pop(n)[1])
    return ex.report()


def test_dock_cmd_forward_and_reverse():
    v, w, ok = dock_cmd((0.0, 0.0, 0.0), (0.5, 0.0), 0.0)
    assert v > 0 and not ok
    v, w, ok = dock_cmd((0.5, 0.0, 0.0), (0.0, 0.0), 0.0)
    assert v < 0 and not ok
    assert dock_cmd((0.5, 0.0, 0.0), (0.503, 0.0), 0.0)[2]


def test_six_chained_tasks_all_done():
    ex = make()
    for b, s in TASKS:
        ex.submit(b, s)
    rep = run(ex, dict(STARTS))
    assert rep["tasks_done"] == 6 and rep["tasks_failed"] == 0
    assert rep["min_separation_m"] >= 0.45
    assert sum(v > 0 for v in rep["per_robot_tasks"].values()) >= 2
    assert rep["boxes"]["box_00"][0] == "shelf_D" and rep["boxes"]["box_03"][0] == "shelf_C"
    m = rep["metrics"]
    assert m["tasks_per_hour"] > 0 and set(m["per_robot"]) == set(STARTS)


def test_failed_pick_frees_robot():
    ex = make()
    ex.submit("box_00", "shelf_D")
    rep = run(ex, dict(STARTS), fail={"box_00"})
    assert rep["tasks_failed"] == 1 and rep["tasks_done"] == 0
    assert all(ph == "idle" for ph, _, _ in ex.phase.values())
    assert ex.tm.pinned == set()


def test_bad_task_rejected():
    ex = make()
    ex.submit("box_99", "shelf_D")
    ex.step(dict(STARTS), 0.1)
    assert len(ex.rejected) == 1 and ex.finished()


def test_lidar_dock_corrects_lateral_offset():
    """measure_fn says the true dock is 0.1 m left of the registry dock (AMCL
    bias); the robot must back up, re-approach and dock on the measured pose."""
    from warehouse_tasks.rack_align import wrap
    at_action = {}

    def measure(n, shelf, slot):
        x, y, yaw = ex.poses[n]
        dx, dy, dyaw = slot["dock"][0] - 0.1 - x, slot["dock"][1] - y, math.radians(slot["dock"][2])
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * dx + s * dy, -s * dx + c * dy, wrap(dyaw - yaw))

    ex = FleetExecutor(make().rm, Registry.load(os.path.join(BR, "config", "registry.yaml")), STARTS,
                       measure_fn=measure)
    ex.submit("box_00", "shelf_D")
    orig = ex.action_done

    def done(n, ok):
        at_action.setdefault(n, ex.poses[n])
        orig(n, ok)
    ex.action_done = done
    rep = run(ex, dict(STARTS))
    assert rep["tasks_done"] == 1
    (n, pose), = list(at_action.items())[:1]
    assert abs(pose[0] - (-1.1)) < 0.03 and abs(pose[1] - 1.5) < 0.03
    assert any("lidar dock" in m for _, m in ex.events)


def test_drop_redocks_to_cancel_held_box_offset():
    calls = []

    def place_error(n, box, slot):
        calls.append(n)
        return (0.08, 0.0) if len(calls) == 1 else (0.005, 0.0)

    ex = FleetExecutor(make().rm, Registry.load(os.path.join(BR, "config", "registry.yaml")), STARTS,
                       place_error_fn=place_error)
    ex.submit("box_00", "shelf_D")
    rep = run(ex, dict(STARTS))
    assert rep["tasks_done"] == 1 and len(calls) == 2   # re-checked after the re-dock
    assert any("re-docking" in m for _, m in ex.events)


def test_drop_redocks_at_most_twice():
    """Day 26: a re-dock can stop short, so the offset is re-checked; at most 2 re-docks."""
    calls = []

    def place_error(n, box, slot):
        calls.append(n)
        return (0.08, 0.0)

    ex = FleetExecutor(make().rm, Registry.load(os.path.join(BR, "config", "registry.yaml")), STARTS,
                       place_error_fn=place_error)
    ex.submit("box_00", "shelf_D")
    rep = run(ex, dict(STARTS))
    assert rep["tasks_done"] == 1
    assert sum("re-docking" in m for _, m in ex.events) == 2


def test_idle_robots_park_off_shelf_fronts():
    """Day 26: after each task the robot parks off the shelf-front cells."""
    ex = make()
    for b, s in TASKS:
        ex.submit(b, s)
    rep = run(ex, dict(STARTS))
    assert rep["tasks_done"] == 6
    assert any("parks at" in m for _, m in ex.events)
    poses = {n: ex.poses[n] for n in ex.poses}
    for _ in range(600):                      # let the last robot finish parking
        for n, (v, w) in ex.step(poses, 0.1).items():
            x, y, th = poses[n]
            poses[n] = (x + v * math.cos(th) * 0.1, y + v * math.sin(th) * 0.1, th + w * 0.1)
    for n, a in ex.tm.agents.items():
        assert a.cell not in ex.shelf_front, (n, a.cell)
    assert rep["min_separation_m"] >= 0.45
