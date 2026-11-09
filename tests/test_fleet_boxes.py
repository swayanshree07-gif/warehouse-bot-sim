import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from fleet_boxes import parse_fboxes  # noqa: E402
from fleet_layout import FLEET_STARTS, parse_starts  # noqa: E402


def test_parse_fboxes():
    d = parse_fboxes("FBOXES t=1.50 robot1:lift=0.030,held=box_00 robot2:lift=0.000,held=- "
                     "box_00=-3.000,2.000,0.480@robot1 box_01=-2.500,2.000,0.450@-")
    assert d["t"] == 1.5
    assert d["robots"]["robot1"] == {"lift": 0.03, "held": "box_00"}
    assert d["robots"]["robot2"]["held"] == "-"
    assert d["boxes"]["box_00"] == ((-3.0, 2.0, 0.48), "robot1")
    assert d["boxes"]["box_01"][1] == "-"


def test_parse_fboxes_rejects_other_lines():
    with pytest.raises(ValueError):
        parse_fboxes("FPOSE t=1 robot1=0,0,0")


def test_parse_starts_overrides_only_given():
    s = parse_starts(["robot2=2.5,1.5,90"])
    assert s["robot2"] == (2.5, 1.5, 90.0)
    assert s["robot1"] == FLEET_STARTS["robot1"]
    assert FLEET_STARTS["robot2"] == (0.0, 0.0, 0.0)   # base not mutated
    with pytest.raises(ValueError):
        parse_starts(["robot9=0,0,0"])
