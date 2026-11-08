import os
import subprocess
import sys
from types import SimpleNamespace as NS

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "ros_ws", "src", "warehouse_tasks"))
from warehouse_tasks.metrics import METRIC_KEYS, compute_metrics  # noqa: E402


def T(state, sub, asg, done, robot):
    return NS(state=state, t_submit=sub, t_assign=asg, t_done=done, robot=robot)


def test_hand_computed():
    tasks = [T("done", 0, 0, 20, "r1"), T("done", 0, 10, 40, "r2"),
             T("failed", 0, 5, 9, "r1")]
    rt = {"r1": {"idle": 30, "busy": 30, "traffic_wait": 2},
          "r2": {"idle": 15, "busy": 45, "traffic_wait": 0}}
    m = compute_metrics(tasks, rt, 60.0)
    assert set(METRIC_KEYS) <= m.keys()
    assert m["tasks_per_hour"] == 120.0
    assert m["mean_task_time_s"] == 25.0 and m["max_task_time_s"] == 30
    assert m["mean_queue_wait_s"] == 5.0
    assert m["per_robot"]["r1"]["idle_pct"] == 50.0
    assert m["per_robot"]["r2"]["idle_pct"] == 25.0
    assert m["mean_idle_pct"] == 37.5
    assert m["per_robot"]["r1"]["tasks_done"] == 1
    assert m["total_traffic_wait_s"] == 2


def test_empty():
    m = compute_metrics([], {}, 0.0)
    assert m["tasks_per_hour"] == 0.0 and m["mean_task_time_s"] == 0.0


def test_lightsim_time_accounting():
    from warehouse_tasks.lightsim import DEFAULT_SCENARIO, LightSim
    from warehouse_tasks.registry import Registry
    from warehouse_tasks.traffic import Roadmap
    br = os.path.join(ROOT, "ros_ws", "src", "warehouse_bringup")
    sim = LightSim(Roadmap.from_map(os.path.join(br, "maps", "warehouse.yaml")),
                   Registry.load(os.path.join(br, "config", "registry.yaml")),
                   {"robot1": (-2.0, 0.0), "robot2": (2.0, 0.0), "robot3": (6.0, 0.0)})
    for b, s in DEFAULT_SCENARIO:
        sim.submit(b, s)
    rep = sim.run()
    m = rep["metrics"]
    for v in m["per_robot"].values():
        assert v["idle_s"] + v["busy_s"] == rep["sim_time_s"]
        assert 0 <= v["traffic_wait_s"] <= v["busy_s"]
    assert sum(v["tasks_done"] for v in m["per_robot"].values()) == 9
    assert m["tasks_per_hour"] >= 200


def test_check_script(tmp_path):
    env = dict(os.environ, OUT=str(tmp_path / "r.json"))
    r = subprocess.run(["timeout", "150", "bash", os.path.join(ROOT, "scripts", "check_fleet_scenario.sh")],
                       env=env, capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "check_fleet_scenario: PASS" in r.stdout


def test_check_script_fails_above_floor(tmp_path):
    env = dict(os.environ, OUT=str(tmp_path / "r.json"), MIN_TPH="100000")
    r = subprocess.run(["timeout", "150", "bash", os.path.join(ROOT, "scripts", "check_fleet_scenario.sh")],
                       env=env, capture_output=True, text=True)
    assert r.returncode != 0
