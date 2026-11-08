"""Day 16: traffic manager - reservations, deadlock detection, priority yield."""
import os
import sys

import pytest

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(ROOT, "ros_ws", "src", "warehouse_tasks"))
from warehouse_tasks.traffic import Roadmap, TrafficManager  # noqa: E402

MAP = os.path.join(ROOT, "ros_ws", "src", "warehouse_bringup", "maps", "warehouse.yaml")


def check_history(history, tm):
    """No two robots share a cell; no two robots swap cells in one tick."""
    prev = None
    for snap in history:
        cells = list(snap.values())
        assert len(cells) == len(set(cells))
        if prev:
            for a in snap:
                for b in snap:
                    if a < b:
                        assert not (snap[a] == prev[b] and snap[b] == prev[a]), "swap"
            for n in snap:  # 4-connected single steps only
                d = abs(snap[n][0] - prev[n][0]) + abs(snap[n][1] - prev[n][1])
                assert d <= 1
        prev = snap


def test_astar_and_blocked():
    rm = Roadmap.from_ascii("""
.....
.###.
.....
""")
    p = rm.plan((0, 1), (4, 1))
    assert p[0] == (0, 1) and p[-1] == (4, 1) and len(p) == 7
    assert rm.plan((0, 1), (4, 1), blocked={(0, 2), (0, 0)}) is None


def test_head_on_swap_corridor_with_bay():
    # 1-wide corridor with one side bay; robots swap ends.
    rm = Roadmap.from_ascii("""
###.###
.......
""")
    tm = TrafficManager(rm)
    tm.add_robot("robot1", (0, 0))
    tm.add_robot("robot2", (6, 0))
    tm.set_goal("robot1", (6, 0))
    tm.set_goal("robot2", (0, 0))
    ticks, hist = tm.run(100)
    check_history(hist, tm)
    assert tm.deadlocks >= 1 and tm.yields >= 1
    assert tm.agents["robot1"].cell == (6, 0) and tm.agents["robot2"].cell == (0, 0)


def test_adjacent_swap_open_area():
    rm = Roadmap.from_ascii("""
....
....
""")
    tm = TrafficManager(rm)
    tm.add_robot("robot1", (1, 0))
    tm.add_robot("robot2", (2, 0))
    tm.set_goal("robot1", (2, 0))
    tm.set_goal("robot2", (1, 0))
    _, hist = tm.run(50)
    check_history(hist, tm)
    assert tm.agents["robot1"].cell == (2, 0) and tm.agents["robot2"].cell == (1, 0)


def test_three_robot_cycle():
    # Ring of 4 cells, 3 robots each wanting the next robot's cell is not a
    # cycle (one free cell); a 4th robot closes it -> 4-cycle; and a 3-cycle
    # case is built from a ring where the 4th cell is a wall-free detour.
    rm = Roadmap.from_ascii("""
....
....
....
""")
    tm = TrafficManager(rm)
    for n, c in (("robot1", (1, 1)), ("robot2", (2, 1)), ("robot3", (2, 2))):
        tm.add_robot(n, c)
    # robot1 -> robot2's cell, robot2 -> robot3's cell, robot3 -> robot1's
    # cell via a 2-step path whose first step is free: the first tick moves
    # robot3, so check the wait-for graph after forcing a 3-cycle directly.
    tm.agents["robot1"].goal, tm.agents["robot1"].path = (2, 1), [(2, 1)]
    tm.agents["robot2"].goal, tm.agents["robot2"].path = (2, 2), [(2, 2)]
    tm.agents["robot3"].goal, tm.agents["robot3"].path = (1, 1), [(1, 1)]  # forced (test only)
    cyc = tm._find_cycle(tm._wait_for())
    assert cyc and set(cyc) == {"robot1", "robot2", "robot3"}
    tm.agents["robot3"].path = [(1, 2), (1, 1)]  # make it a legal route
    _, hist = tm.run(100)
    check_history(hist, tm)
    assert {n: a.cell for n, a in tm.agents.items()} == {
        "robot1": (2, 1), "robot2": (2, 2), "robot3": (1, 1)}


def test_four_robot_ring_deadlock_resolved():
    rm = Roadmap.from_ascii("""
....
....
....
""")
    tm = TrafficManager(rm)
    ring = [(1, 1), (2, 1), (2, 2), (1, 2)]
    for i, c in enumerate(ring, 1):
        tm.add_robot(f"robot{i}", c)
    for i in range(4):
        a = tm.agents[f"robot{i + 1}"]
        a.goal, a.path = ring[(i + 1) % 4], [ring[(i + 1) % 4]]
    cyc = tm._find_cycle(tm._wait_for())
    assert cyc and len(cyc) == 4
    _, hist = tm.run(100)
    check_history(hist, tm)
    assert tm.deadlocks >= 1
    assert [tm.agents[f"robot{i + 1}"].cell for i in range(4)] == ring[1:] + ring[:1]


def test_narrow_aisle_three_robots():
    # 1-wide aisle (middle row) between two shelves, open rows above/below.
    rm = Roadmap.from_ascii("""
.........
##.....##
.........
""")
    tm = TrafficManager(rm)
    tm.add_robot("robot1", (2, 1))
    tm.add_robot("robot2", (6, 1))
    tm.add_robot("robot3", (5, 1))
    tm.set_goal("robot1", (6, 1))
    tm.set_goal("robot2", (2, 1))
    tm.set_goal("robot3", (3, 1))
    _, hist = tm.run(200)
    check_history(hist, tm)
    assert {n: a.cell for n, a in tm.agents.items()} == {
        "robot1": (6, 1), "robot2": (2, 1), "robot3": (3, 1)}


def test_single_lane_aisle_with_bay():
    # Strict 1-wide aisle (y=0) with two bays (one bay per extra robot is
    # needed for a 3-way crossing): three robots crossing.
    rm = Roadmap.from_ascii("""
##.###.##
.........
""")
    tm = TrafficManager(rm)
    tm.add_robot("robot1", (0, 0))
    tm.add_robot("robot2", (8, 0))
    tm.add_robot("robot3", (7, 0))
    tm.set_goal("robot1", (7, 0))
    tm.set_goal("robot2", (0, 0))
    tm.set_goal("robot3", (8, 0))
    _, hist = tm.run(300)
    check_history(hist, tm)
    assert tm.all_done()


def test_parked_robot_is_routed_around():
    rm = Roadmap.from_ascii("""
.....
.....
""")
    tm = TrafficManager(rm)
    tm.add_robot("robot1", (0, 0))
    tm.add_robot("parked", (2, 0))  # no goal: stands still
    tm.set_goal("robot1", (4, 0))
    _, hist = tm.run(60)
    check_history(hist, tm)
    assert tm.agents["robot1"].cell == (4, 0)


def test_roadmap_from_saved_map():
    rm = Roadmap.from_map(MAP, cell=0.5, inflate=0.3)
    assert 60 < len(rm.free) < rm.cols * rm.rows
    # cells near the map origin corner are walls, centre of warehouse is free
    a = rm.nearest_free(rm.to_cell(-0.5, 0.0))
    b = rm.nearest_free(rm.to_cell(4.0, 0.0))
    assert rm.plan(a, b) is not None
    assert rm.path_length((-0.5, 0.0), (4.0, 0.0)) >= 4.0


def test_three_robots_on_saved_map_cross():
    rm = Roadmap.from_map(MAP)
    big = max(_components(rm), key=len)
    cells = sorted(big)
    starts = [cells[0], cells[len(cells) // 2], cells[-1]]
    goals = [cells[-1], cells[0], cells[len(cells) // 4]]
    tm = TrafficManager(rm)
    for i, (s, g) in enumerate(zip(starts, goals), 1):
        tm.add_robot(f"robot{i}", s)
    for i, g in enumerate(goals, 1):
        tm.set_goal(f"robot{i}", g)
    _, hist = tm.run(500)
    check_history(hist, tm)
    assert all(tm.agents[f"robot{i}"].cell == g for i, g in enumerate(goals, 1))


def _components(rm):
    seen, comps = set(), []
    for c in rm.free:
        if c in seen:
            continue
        comp, stack = set(), [c]
        while stack:
            x = stack.pop()
            if x in comp:
                continue
            comp.add(x)
            stack.extend(n for n in rm.neighbors(x) if n not in comp)
        seen |= comp
        comps.append(comp)
    return comps


def test_bad_inputs():
    rm = Roadmap.from_ascii("..#")
    tm = TrafficManager(rm)
    tm.add_robot("a", (0, 0))
    with pytest.raises(ValueError):
        tm.add_robot("b", (0, 0))
    with pytest.raises(ValueError):
        tm.add_robot("c", (2, 0))
    with pytest.raises(ValueError):
        tm.set_goal("a", (2, 0))


def test_random_scenarios_on_saved_map():
    """60 seeded random 3-robot start/goal sets on the real map: all finish,
    no shared cells, no swaps."""
    import random
    rm = Roadmap.from_map(MAP)
    cells = sorted(max(_components(rm), key=len))
    fails = []
    for seed in range(60):
        rnd = random.Random(seed)
        pts = rnd.sample(cells, 6)
        tm = TrafficManager(rm)
        for i in range(3):
            tm.add_robot(f"robot{i + 1}", pts[i])
        for i in range(3):
            tm.set_goal(f"robot{i + 1}", pts[3 + i])
        try:
            _, hist = tm.run(400)
            check_history(hist, tm)
        except RuntimeError as e:
            fails.append((seed, str(e)))
    assert not fails, fails
