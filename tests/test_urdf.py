"""The generated URDF matches robot_params and the checked-in file is fresh."""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
import robot_params as P  # noqa: E402
import urdf_gen  # noqa: E402


def _xyz(j):
    return tuple(float(v) for v in j.find("origin").get("xyz").split())


def test_checked_in_urdf_is_fresh():
    assert Path(urdf_gen.URDF_PATH).read_text() == urdf_gen.generate()


def test_joints_match_params():
    root = ET.fromstring(urdf_gen.generate())
    joints = {j.get("name"): j for j in root.findall("joint")}
    links = {link.get("name") for link in root.findall("link")}
    assert {"base_link", "base_footprint", "laser", "lift_link", "left_wheel_link", "right_wheel_link"} <= links
    cz = P.chassis_centre_z()
    for side, sy in (("left", 1), ("right", -1)):
        j = joints[f"{side}_wheel_joint"]
        assert j.get("type") == "continuous"
        x, y, z = _xyz(j)
        assert abs(y - sy * P.WHEEL_SEPARATION / 2) < 1e-4 and abs(z - (P.WHEEL_RADIUS - cz)) < 1e-4
        assert j.find("axis").get("xyz") == "0 1 0"
    assert all(abs(a - b) < 1e-4 for a, b in zip(_xyz(joints["laser_joint"]), P.LASER_POS))
    lift = joints["lift_joint"]
    assert lift.get("type") == "prismatic"
    lim = lift.find("limit")
    assert float(lim.get("lower")) == P.LIFT_TRAVEL[0] and float(lim.get("upper")) == P.LIFT_TRAVEL[1]
    # every joint's parent/child exists, and each link has at most one parent
    children = [j.find("child").get("link") for j in joints.values()]
    assert len(children) == len(set(children))
    assert all(j.find("parent").get("link") in links and j.find("child").get("link") in links
               for j in joints.values())
