import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros_ws/src/warehouse_tasks"))
from warehouse_tasks.path_follower import FleetFollower, cmd_to  # noqa: E402
from warehouse_tasks.traffic import Roadmap  # noqa: E402

MAP = os.path.join(ROOT, "ros_ws/src/warehouse_bringup/maps/warehouse.yaml")


def test_cmd_turns_in_place_then_drives():
    v, w, ok = cmd_to((0, 0, 0), (0, 1))
    assert v == 0 and w > 0 and not ok
    v, w, ok = cmd_to((0, 0, 0), (1, 0))
    assert v > 0 and abs(w) < 1e-9
    assert cmd_to((0, 0, 0), (0.01, 0))[2]


def simulate(ff, poses, dt=0.05, steps=40000):
    min_sep = 9.0
    for _ in range(steps):
        cmds = ff.step(poses)
        if ff.done() and all(v == 0 and w == 0 for v, w in cmds.values()):
            return min_sep
        for n, (v, w) in cmds.items():
            x, y, t = poses[n]
            poses[n] = (x + v * math.cos(t) * dt, y + v * math.sin(t) * dt, t + w * dt)
        ps = list(poses.values())
        for i in range(len(ps)):
            for j in range(i + 1, len(ps)):
                min_sep = min(min_sep, math.hypot(ps[i][0] - ps[j][0], ps[i][1] - ps[j][1]))
    raise AssertionError("fleet did not finish")


def test_three_robots_reach_crossing_goals():
    rm = Roadmap.from_map(MAP)
    starts = {"robot1": (-1.0, 0.0), "robot2": (2.0, 0.0), "robot3": (5.0, 0.0)}
    goals = {"robot1": (4.75, 3.0), "robot2": (2.0, -3.0), "robot3": (-1.28, -2.78)}
    ff = FleetFollower(rm, starts)
    for n, g in goals.items():
        ff.set_goal(n, g)
    poses = {n: (x, y, 0.0) for n, (x, y) in starts.items()}
    min_sep = simulate(ff, poses)
    for n, g in goals.items():
        assert math.hypot(poses[n][0] - g[0], poses[n][1] - g[1]) < 0.4, n
    assert min_sep >= 0.45


def test_head_on_swap_in_aisle():
    rm = Roadmap.from_map(MAP)
    starts = {"robot1": (-1.0, 0.0), "robot3": (5.0, 0.0)}
    ff = FleetFollower(rm, starts)
    ff.set_goal("robot1", (5.0, 0.0))
    ff.set_goal("robot3", (-1.0, 0.0))
    poses = {n: (x, y, 0.0) for n, (x, y) in starts.items()}
    assert simulate(ff, poses) >= 0.45
