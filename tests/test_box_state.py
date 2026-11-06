"""Day 22/23: ground-truth box classification (sim/box_state.py), the task
manager's belief sync (Registry.sync / slot_at) and the missing-box path."""
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "ros_ws" / "src" / "warehouse_tasks"))
import box_state as B  # noqa: E402
import registry_gen as R  # noqa: E402
import warehouse_layout as L  # noqa: E402
from warehouse_tasks.mission import run_move_box  # noqa: E402
from warehouse_tasks.mock_robot import MockRobot  # noqa: E402
from warehouse_tasks.registry import Registry, TaskError  # noqa: E402

REG_PATH = os.path.join(ROOT, "ros_ws", "src", "warehouse_bringup", "config", "registry.yaml")
GEN = R.build()
TABLE = B.slot_table(GEN)
TO_MAP = B.world_to_map_fn(GEN["map_origin_in_world"])


@pytest.fixture
def reg():
    return Registry.load(REG_PATH)


def test_initial_layout_boxes_classify_to_their_slots():
    for b in L.BOXES:
        assert B.classify(TO_MAP((b.x, b.y, b.z)), TABLE) == (b.shelf, b.slot), b.name


def test_classify_rejects_held_lifted_floor_and_between():
    c = GEN["shelves"]["shelf_A"]["slots"][1]["center"]
    assert B.classify(c, TABLE) == ("shelf_A", 1)
    assert B.classify(c, TABLE, held=True) == ("", -1)
    assert B.classify((c[0], c[1], c[2] + 0.03), TABLE) == ("", -1)       # lifted 3 cm
    assert B.classify((c[0] + 0.25, c[1], c[2]), TABLE) == ("", -1)       # between slots
    assert B.classify((c[0] + 0.08, c[1], c[2]), TABLE) == ("shelf_A", 1)  # within 10 cm
    assert B.classify(TO_MAP((4.6, -3.6, 0.1)), TABLE) == ("", -1)         # /sim/remove_box spot


def test_registry_slot_at_matches_sim_classifier(reg):
    for shelf, idx, c in TABLE:
        for dx, dz in ((0, 0), (0.07, 0), (0.12, 0), (0, 0.03)):
            p = (c[0] + dx, c[1], c[2] + dz)
            want = B.classify(p, TABLE)
            got = reg.slot_at(p)
            assert (got[0] or "", got[1]) == want


def test_sync_marks_missing_and_held(reg):
    states = {b: (s, i, False) for b, (s, i) in reg.boxes.items()}
    states["box_03"] = ("", -1, False)
    states["box_04"] = ("", -1, True)
    changes = reg.sync(states)
    assert len(changes) == 2
    with pytest.raises(TaskError, match=r"box_03 is not on any shelf slot \(missing\)"):
        reg.plan("box_03", "shelf_D")
    with pytest.raises(TaskError, match=r"held by the robot"):
        reg.plan("box_04", "shelf_D")
    # the slots they left are free now
    old = Registry.load(REG_PATH).boxes["box_03"]
    assert reg.occupant(*old) is None


def test_sync_moves_box_to_ground_truth_slot(reg):
    src = reg.boxes["box_09"]
    free = ("shelf_D", reg.free_slots("shelf_D")[0])
    assert reg.sync({"box_09": (free[0], free[1], False)}) == [f"box_09: {src} -> {free}"]
    assert reg.occupant(*free) == "box_09"
    assert reg.occupant(*src) is None


def test_mock_missing_box_rejected_after_sync(reg):
    robot = MockRobot(reg, missing=["box_03"])
    reg.sync(robot.box_states())
    with pytest.raises(TaskError, match="missing"):
        reg.plan("box_03", "shelf_D")
    # the other boxes still plan and run, with a real place error from the mock
    plan = reg.plan("box_09", "shelf_D")
    out = run_move_box(robot, plan, reg.lift_clearance)
    assert out.success and out.place_error == pytest.approx(0.0, abs=1e-6)
    reg.sync(robot.box_states())
    assert reg.boxes["box_09"] == ("shelf_D", plan["dst_slot"])


def test_mock_box_states_match_registry_initially(reg):
    st = MockRobot(reg).box_states()
    assert {b: (s, i) for b, (s, i, _h) in st.items()} == reg.boxes
