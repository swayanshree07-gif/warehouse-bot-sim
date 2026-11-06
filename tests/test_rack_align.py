"""Day 20: lidar rack-face measurement on perfect synthetic scans (sim/geometry2d)."""
import math
import os
import random
import sys

import pytest

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "ros_ws", "src", "warehouse_tasks"))
import geometry2d as G  # noqa: E402
import registry_gen as R  # noqa: E402
import robot_params as P  # noqa: E402
from warehouse_tasks.rack_align import (compose, dock_in_base, measure_face,  # noqa: E402
                                        slot_geometry, wrap)

REG = R.load()
OX, OY, _ = REG["map_origin_in_world"]
ANG = P.laser_angles_deg()
INC = math.radians(P.LASER_RES_DEG)
SLOTS = [(name, s) for name, sh in REG["shelves"].items() for s in sh["slots"]]


def scan_at(pose_w):
    """Perfect scan (laser frame) for base_link at world pose (x, y, yaw rad)."""
    lx, ly, _ = P.LASER_POS
    x = pose_w[0] + lx * math.cos(pose_w[2]) - ly * math.sin(pose_w[2])
    y = pose_w[1] + lx * math.sin(pose_w[2]) + ly * math.cos(pose_w[2])
    return G.expected_scan(x, y, pose_w[2], ANG, P.laser_height(), P.LASER_RANGE[1])


def perturb(pose_map, along, lat, dyaw_deg):
    yaw = math.radians(pose_map[2])
    ux, uy = math.cos(yaw), math.sin(yaw)
    return (pose_map[0] + OX + along * ux - lat * uy, pose_map[1] + OY + along * uy + lat * ux,
            yaw + math.radians(dyaw_deg))


def measure(pose_w, shelf, slot, ranges=None):
    ranges = ranges if ranges is not None else scan_at(pose_w)
    lat, depth, flen = slot_geometry(REG["shelves"][shelf], slot, slot["dock"][2])
    meas, why = measure_face(ranges, math.radians(ANG[0]), INC, P.LASER_RANGE[0], P.LASER_RANGE[1],
                             laser=(P.LASER_POS[0], P.LASER_POS[1], 0.0),
                             expect_dist=P.APPROACH_STANDOFF - depth / 2, face_len=flen)
    if meas is None:
        return None, why
    rel, err = dock_in_base(meas, lat, depth, P.DOCK_STANDOFF)
    return (compose(pose_w, rel), err), ""


@pytest.mark.parametrize("shelf,slot", SLOTS, ids=[f"{n}:{s['index']}" for n, s in SLOTS])
def test_exact_at_approach(shelf, slot):
    pose = perturb(slot["approach"], 0, 0, 0)
    (dock_w, err), why = measure(pose, shelf, slot)
    assert why == ""
    want = (slot["dock"][0] + OX, slot["dock"][1] + OY)
    assert math.hypot(dock_w[0] - want[0], dock_w[1] - want[1]) < 0.015
    assert abs(math.degrees(wrap(dock_w[2] - math.radians(slot["dock"][2])))) < 0.3
    assert err["along"] == pytest.approx(P.APPROACH_STANDOFF - P.DOCK_STANDOFF, abs=0.005)


def test_random_offsets_all_slots():
    """Nav2 leaves the robot up to ~0.13 m / 8 deg off the approach pose."""
    rng = random.Random(7)
    worst_xy = worst_yaw = 0.0
    for shelf, slot in SLOTS:
        for _ in range(8):
            a, l, dy = rng.uniform(-0.13, 0.13), rng.uniform(-0.13, 0.13), rng.uniform(-8, 8)
            pose = perturb(slot["approach"], a, l, dy)
            (dock_w, err), why = measure(pose, shelf, slot)
            assert why == "", (shelf, slot["index"], a, l, dy, why)
            want = (slot["dock"][0] + OX, slot["dock"][1] + OY)
            worst_xy = max(worst_xy, math.hypot(dock_w[0] - want[0], dock_w[1] - want[1]))
            worst_yaw = max(worst_yaw, abs(math.degrees(wrap(dock_w[2] - math.radians(slot["dock"][2])))))
            # the reported errors describe the true offset (lateral = robot left of slot line)
            assert err["lateral"] == pytest.approx(l, abs=0.02)
            assert math.degrees(err["yaw"]) == pytest.approx(dy, abs=0.3)
    assert worst_xy < 0.02 and worst_yaw < 0.3, (worst_xy, worst_yaw)


def test_hidden_corner_rejected():
    shelf, slot = SLOTS[0]
    pose = perturb(slot["approach"], 0, 0, 0)
    ranges = scan_at(pose)
    n = len(ranges)
    # blank out beams beyond +15 deg (left side): left corner never seen
    ranges = [r if not (15 <= ANG[i] <= 180) else math.inf for i, r in enumerate(ranges)]
    assert n == len(ranges)
    res, why = measure(pose, shelf, slot, ranges)
    assert res is None and ("corner" in why or "face length" in why)


def test_no_rack_rejected():
    res, why = measure((0.0 + OX, 0.0, 0.0), *SLOTS[0], ranges=[math.inf] * len(ANG))
    assert res is None and "scan points" in why


def test_compose():
    p = compose((1.0, 2.0, math.pi / 2), (0.5, 0.1, 0.2))
    assert p[0] == pytest.approx(0.9) and p[1] == pytest.approx(2.5) and p[2] == pytest.approx(math.pi / 2 + 0.2)
