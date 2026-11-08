"""Day 17: light 2D fleet simulator - 3 robots, 9 tasks, separation."""
import os
import random
import subprocess
import sys

import pytest

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(ROOT, "ros_ws", "src", "warehouse_tasks"))
from warehouse_tasks.lightsim import DEFAULT_SCENARIO, LightSim  # noqa: E402
from warehouse_tasks.registry import Registry  # noqa: E402
from warehouse_tasks.traffic import Roadmap  # noqa: E402

BR = os.path.join(ROOT, "ros_ws", "src", "warehouse_bringup")
STARTS = {"robot1": (-2.0, 0.0), "robot2": (2.0, 0.0), "robot3": (6.0, 0.0)}


@pytest.fixture(scope="module")
def rm():
    return Roadmap.from_map(os.path.join(BR, "maps", "warehouse.yaml"))


@pytest.fixture
def reg():
    return Registry.load(os.path.join(BR, "config", "registry.yaml"))


def run(rm, reg, starts, scenario=DEFAULT_SCENARIO):
    sim = LightSim(rm, reg, starts)
    for b, s in scenario:
        sim.submit(b, s)
    return sim, sim.run(max_time=1500)


def test_default_scenario(rm, reg):
    sim, rep = run(rm, reg, STARTS)
    assert rep["tasks_done"] == 9 and rep["tasks_failed"] == 0 and rep["rejected"] == 0
    assert rep["collisions"] == 0 and rep["min_separation_m"] >= 0.5
    assert all(n >= 1 for n in rep["per_robot_tasks"].values())   # all 3 used
    boxes = sim.fleet.registry.boxes
    assert boxes["box_00"][0] == "zone_E" and boxes["box_09"][0] == "shelf_B"
    assert len(set(boxes.values())) == len(boxes)                   # one box per slot
    assert reg.boxes["box_00"] == ("shelf_A", 0)                    # input not mutated


def test_three_tasks_run_concurrently(rm, reg):
    sim = LightSim(rm, reg, STARTS)
    for b, s in DEFAULT_SCENARIO:
        sim.submit(b, s)
    sim.step()
    assert sum(ph != "idle" for ph, _, _ in sim.phase.values()) == 3


@pytest.mark.parametrize("seed", range(15))
def test_random_starts_and_order(rm, reg, seed):
    rnd = random.Random(seed)
    cells = rnd.sample(sorted(rm.free), 3)
    starts = {f"robot{i + 1}": rm.to_world(c) for i, c in enumerate(cells)}
    scen = list(DEFAULT_SCENARIO)
    rnd.shuffle(scen)
    sim, rep = run(rm, reg, starts, scen)
    assert rep["tasks_done"] == 9 and rep["collisions"] == 0
    assert rep["min_separation_m"] >= 0.5 - 1e-9


def test_separation_detects_overlap(rm, reg):
    sim = LightSim(rm, reg, STARTS)
    sim._check_separation({"a": (0, 0), "b": (1, 0)}, {"a": (0.5, 0), "b": (0.5, 0)})
    assert sim.collisions >= 1 and sim.min_sep == 0


def test_cli_under_timeout(tmp_path):
    out = tmp_path / "r.json"
    p = subprocess.run(["timeout", "120", sys.executable,
                        os.path.join(ROOT, "scripts", "run_lightsim.py"), "--out", str(out)],
                       capture_output=True, text=True)
    assert p.returncode == 0, p.stdout + p.stderr
    assert "PASS" in p.stdout and out.exists()
