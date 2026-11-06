"""The health check must expect exactly the lifecycle nodes nav2.launch.py starts (Day 24)."""
import ast
import os

ROOT = os.path.join(os.path.dirname(__file__), "..")


def _lists(path):
    tree = ast.parse(open(path).read())
    out = {}
    for n in tree.body:
        if isinstance(n, ast.Assign) and isinstance(n.value, ast.List):
            out[n.targets[0].id] = [e.value for e in n.value.elts]
    return out


def test_navigation_nodes_match_launch():
    launch = _lists(os.path.join(ROOT, "ros_ws", "src", "warehouse_bringup", "launch", "nav2.launch.py"))
    hc = _lists(os.path.join(ROOT, "scripts", "health_check.py"))
    assert hc["NAVIGATION"] == launch["NAVIGATION"]


def test_localization_nodes_match_launch():
    launch = _lists(os.path.join(ROOT, "ros_ws", "src", "warehouse_bringup", "launch", "nav2.launch.py"))
    src = open(os.path.join(ROOT, "scripts", "health_check.py")).read()
    assert '["map_server", "amcl"]' in src and launch["LOCALIZATION"] == ["map_server", "amcl"]
    assert '["slam_toolbox"]' in src
