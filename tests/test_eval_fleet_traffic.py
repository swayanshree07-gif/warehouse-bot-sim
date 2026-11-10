import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from eval_fleet_traffic import evaluate, min_separation  # noqa: E402

ident = lambda x, y: (x, y)  # noqa: E731
G = {"a": (2.0, 0.0), "b": (0.0, 0.0)}
SIM = "FPOSE t=0 a=0,0,0 b=2,0,0\n" * 10 + "FPOSE t=9 a=2,0,0 b=0,0.5,0\n" * 2


def test_min_sep():
    assert abs(min_separation([{"a": (0, 0), "b": (3, 4), "c": (0, 1)}]) - 1.0) < 1e-9


def test_pass():
    r = evaluate(SIM, "FOLLOW_DONE ticks=5 secs=9 deadlocks=1", {"a": (2, 0), "b": (0, 0.5)}, 1000, 5000, ident)
    assert r["ok"] and abs(r["min_sep"] - 2.0) < 1e-9


def test_fail_too_close_or_no_deadlock():
    close = SIM + "FPOSE t=5 a=1,0,0 b=1.3,0,0\n"
    assert not evaluate(close, "FOLLOW_DONE deadlocks=1", {"a": (2, 0), "b": (0, 0.5)}, 1, 1, ident)["ok"]
    assert not evaluate(SIM, "FOLLOW_DONE deadlocks=0", {"a": (2, 0), "b": (0, 0.5)}, 1, 1, ident)["ok"]
    assert not evaluate(SIM, "FOLLOW_TIMEOUT deadlocks=1", {"a": (2, 0), "b": (0, 0.5)}, 1, 1, ident)["ok"]
