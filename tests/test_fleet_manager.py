"""Day 23: pure helpers of the fleet_manager node (no ROS spin)."""
import json
import os
import sys

import pytest

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(ROOT, "ros_ws", "src", "warehouse_tasks"))
fm = pytest.importorskip("warehouse_tasks.fleet_manager")  # needs rclpy (env.sh)
from warehouse_tasks.lightsim import LightSim  # noqa: E402
from warehouse_tasks.registry import Registry  # noqa: E402
from warehouse_tasks.traffic import Roadmap  # noqa: E402

BR = os.path.join(ROOT, "ros_ws", "src", "warehouse_bringup")


def test_parse_starts_and_task():
    assert fm.parse_starts(fm.DEFAULT_STARTS) == {
        "robot1": (-2.0, 0.0), "robot2": (2.0, 0.0), "robot3": (6.0, 0.0)}
    assert fm.parse_task(" box_00  zone_E ") == ("box_00", "zone_E")
    with pytest.raises(ValueError):
        fm.parse_task("garbage")


def test_status_dict_tracks_assignments():
    sim = LightSim(Roadmap.from_map(os.path.join(BR, "maps", "warehouse.yaml")),
                   Registry.load(os.path.join(BR, "config", "registry.yaml")),
                   fm.parse_starts(fm.DEFAULT_STARTS))
    s0 = fm.status_dict(sim)
    assert s0["min_separation_m"] is None and s0["metrics"] == {}
    sim.submit("box_00", "zone_E")
    sim.submit("box_99", "zone_E")
    sim.step()
    s = json.loads(json.dumps(fm.status_dict(sim)))   # must be JSON-safe
    assert s["tasks"]["1"]["robot"] in s["robots"]
    assert s["robots"][s["tasks"]["1"]["robot"]]["state"] == "busy"
    assert len(s["rejected"]) == 1
    sim.run()
    s = fm.status_dict(sim)
    assert s["done"] == 1 and s["tasks"]["1"]["state"] == "done"
    assert "tasks_per_hour" in s["metrics"]
