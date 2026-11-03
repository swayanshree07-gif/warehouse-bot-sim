"""Nav2 config (Day 12) stays consistent with the robot, sensor and launch file."""
import ast
import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros_ws" / "src" / "warehouse_bringup"
sys.path.insert(0, str(ROOT / "sim"))
import map_eval as M  # noqa: E402
import robot_params as P  # noqa: E402

CFG = yaml.safe_load((PKG / "config" / "nav2.yaml").read_text())


def _params(name):
    sec = CFG[name]
    return sec[name]["ros__parameters"] if name.endswith("costmap") else sec["ros__parameters"]


def test_costmap_footprints_match_robot():
    want = [list(p) for p in P.footprint()]
    for cm in ("local_costmap", "global_costmap"):
        assert ast.literal_eval(_params(cm)["footprint"]) == want


def test_laser_range_matches_sensor():
    amcl = _params("amcl")
    assert amcl["laser_max_range"] == P.LASER_RANGE[1]
    assert amcl["laser_min_range"] == P.LASER_RANGE[0]


def test_controller_respects_robot_limits_and_publishes_twist():
    ctl = _params("controller_server")
    assert ctl["enable_stamped_cmd_vel"] is False
    assert _params("behavior_server")["enable_stamped_cmd_vel"] is False
    assert ctl["FollowPath"]["desired_linear_vel"] <= P.MAX_LINEAR_SPEED


def test_launch_lifecycle_nodes_have_params():
    src = (PKG / "launch" / "nav2.launch.py").read_text()
    names = re.findall(r'"([a-z_]+)"', src.split("def generate_launch_description")[0])
    assert {"map_server", "amcl", "controller_server", "bt_navigator"} <= set(names)
    for n in names:
        assert n in CFG, f"{n} has no section in nav2.yaml"


def test_packaged_map_is_valid():
    g = M.GridMap(str(PKG / "maps" / "warehouse.yaml"))
    assert abs(g.res - 0.05) < 1e-9
    cov, _ = M.coverage(g, (-2.0, 0.0, 0.0))
    assert cov > 0.95
    assert M.occupied_iou(g, (-2.0, 0.0, 0.0))["iou"] > 0.7
