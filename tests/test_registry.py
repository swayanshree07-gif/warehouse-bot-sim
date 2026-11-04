"""Day 15: the shelf/box registry agrees with the scene builder's layout and
its robot poses are usable (inside the room, collision free, free on the map).
Run: python3 -m pytest -q tests/
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
import geometry2d as G  # noqa: E402
import map_eval  # noqa: E402
import registry_gen as R  # noqa: E402
import robot_params as P  # noqa: E402
import warehouse_layout as L  # noqa: E402

REG = R.load()
MAP_YAML = ROOT / "ros_ws" / "src" / "warehouse_bringup" / "maps" / "warehouse.yaml"


def _to_world(mx, my):
    return mx + R.SPAWN_WORLD[0], my + R.SPAWN_WORLD[1]


def _footprint_world(pose, pad=0.0):
    """Axis-aligned bounds of the (pad-inflated) footprint at a map pose.
    Registry yaws are multiples of 90 deg, so the bounds are exact."""
    x, y = _to_world(pose[0], pose[1])
    yaw = math.radians(pose[2])
    pts = [(x + math.cos(yaw) * (fx + math.copysign(pad, fx)) - math.sin(yaw) * (fy + math.copysign(pad, fy)),
            y + math.sin(yaw) * (fx + math.copysign(pad, fx)) + math.cos(yaw) * (fy + math.copysign(pad, fy)))
           for fx, fy in P.footprint()]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def _overlap(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def _slots():
    for name, s in REG["shelves"].items():
        for slot in s["slots"]:
            yield name, slot


def test_checked_in_registry_is_fresh():
    assert Path(R.REGISTRY_PATH).read_text() == R.generate()


def test_registry_header_fields():
    assert REG["frame"] == "map"
    assert REG["map_origin_in_world"] == list(R.SPAWN_WORLD)
    assert REG["gripper"]["reach"] == P.GRIP_REACH
    assert abs(REG["box_size"] - L.BOX_SIZE) < 1e-9


def test_shelves_and_slots_match_layout():
    layout = {s.name: s for s in L.SHELVES}
    assert set(REG["shelves"]) == set(layout)
    for name, s in REG["shelves"].items():
        ls = layout[name]
        assert _to_world(*s["center"]) == (ls.x, ls.y)
        assert len(s["slots"]) == L.SLOTS_PER_SHELF
        for slot, (x, y, ztop) in zip(s["slots"], ls.slots):
            wx, wy = _to_world(slot["center"][0], slot["center"][1])
            assert abs(wx - x) < 1e-3 and abs(wy - y) < 1e-3
            assert abs(slot["center"][2] - (ztop + L.BOX_SIZE / 2)) < 1e-3


def test_boxes_match_layout_and_slots_unique():
    assert set(REG["boxes"]) == {b.name for b in L.BOXES}
    for b in L.BOXES:
        assert REG["boxes"][b.name] == {"shelf": b.shelf, "slot": b.slot}
        slot = REG["shelves"][b.shelf]["slots"][b.slot]
        wx, wy = _to_world(slot["center"][0], slot["center"][1])
        assert abs(wx - b.x) < 1e-3 and abs(wy - b.y) < 1e-3 and abs(slot["center"][2] - b.z) < 1e-3
    used = [(v["shelf"], v["slot"]) for v in REG["boxes"].values()]
    assert len(set(used)) == len(used)
    n_slots = sum(len(s["slots"]) for s in REG["shelves"].values())
    assert n_slots - len(used) >= 1, "need at least one free slot to move a box to"


def test_dock_pose_puts_grip_point_on_slot_centre():
    for _, slot in _slots():
        x, y, yaw = slot["dock"]
        gx = x + P.GRIP_REACH * math.cos(math.radians(yaw))
        gy = y + P.GRIP_REACH * math.sin(math.radians(yaw))
        assert math.hypot(gx - slot["center"][0], gy - slot["center"][1]) < 1e-3


def test_poses_face_their_slot_from_the_aisle():
    for _, slot in _slots():
        for key in ("approach", "dock"):
            x, y, yaw = slot[key]
            bearing = math.degrees(math.atan2(slot["center"][1] - y, slot["center"][0] - x))
            assert abs((bearing - yaw + 180) % 360 - 180) < 0.1
            # served from the central aisle side
            assert abs(_to_world(x, y)[1]) < abs(_to_world(*slot["center"][:2])[1])


def test_dock_footprint_clears_racks_and_walls():
    rects = G.obstacles_at_height(0.1)   # chassis height: walls + racks
    for _, slot in _slots():
        fp = _footprint_world(slot["dock"], pad=0.02)
        assert not any(_overlap(fp, r) for r in rects), slot


def test_approach_footprint_has_margin():
    rects = G.obstacles_at_height(0.1)
    for _, slot in _slots():
        fp = _footprint_world(slot["approach"], pad=0.3)
        assert not any(_overlap(fp, r) for r in rects), slot


def test_carried_box_clears_chassis_and_lidar():
    # the box hangs GRIP_REACH ahead of base_link: it must not touch the chassis
    # front and must stay above the scan plane
    assert P.GRIP_REACH - L.BOX_SIZE / 2 > P.CHASSIS_SIZE[0] / 2 + 0.03
    box_bottom = L.SHELF_SIZE[2] + P.LIFT_CLEARANCE
    assert box_bottom > P.laser_height() + 0.05
    assert P.LIFT_CLEARANCE < P.LIFT_TRAVEL[1]


def test_approach_poses_free_on_packaged_map():
    grid = map_eval.GridMap(str(MAP_YAML))
    for _, slot in _slots():
        for key in ("approach", "dock"):
            x, y, _ = slot[key]
            assert grid.value_at(x, y) == 254, (key, slot)
