"""Day 20: docking line-tracking law on an ideal unicycle (no sim)."""
import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros_ws", "src", "warehouse_tasks"))
from warehouse_tasks.line_track import errors, track_cmd, wrap  # noqa: E402


def simulate(pose, target, yaw, reverse, noise=0.0, dt=0.03, t_max=40.0, seed=0):
    rng = random.Random(seed)
    x, y, th = pose
    t = 0.0
    while t < t_max:
        meas = (x + rng.gauss(0, noise), y + rng.gauss(0, noise), th + rng.gauss(0, noise))
        v, w, done, _, _ = track_cmd(meas, target, yaw, reverse=reverse)
        if done:
            return (x, y, th), t
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th = wrap(th + w * dt)
        t += dt
    return None, t


def test_errors_signs():
    yaw = -math.pi / 2   # dock heading -90 deg (rack to the -y side)
    rem, lat = errors((4.0, -1.1, yaw), (4.0, -1.5), yaw)
    assert rem == pytest.approx(0.4) and lat == pytest.approx(0.0)
    _, lat = errors((4.05, -1.1, yaw), (4.0, -1.5), yaw)   # +x is the robot's left when facing -y
    assert lat == pytest.approx(0.05)


@pytest.mark.parametrize("dx,dyaw_deg", [(0.0, 0.0), (0.06, 0.0), (-0.08, 8.0), (0.05, -10.0), (0.0, 20.0)])
def test_dock_converges_from_offset(dx, dyaw_deg):
    yaw = -math.pi / 2
    start = (4.0 + dx, -1.1 + 0.13, yaw + math.radians(dyaw_deg))   # Nav2 stops ~0.13 m short
    end, t = simulate(start, (4.0, -1.5), yaw, reverse=False)
    assert end is not None, "did not finish"
    rem, lat = errors(end, (4.0, -1.5), yaw)
    assert abs(rem) < 0.01 and abs(lat) < 0.02, (rem, lat)
    assert t < 20


def test_dock_with_pose_noise_no_stall():
    yaw = math.pi / 2
    for seed in range(5):
        end, _ = simulate((-0.45, 1.05, yaw + 0.1), (-0.5, 1.5), yaw, reverse=False, noise=0.01, seed=seed)
        assert end is not None
        assert abs(errors(end, (-0.5, 1.5), yaw)[1]) < 0.03


def test_undock_reverses_to_approach():
    yaw = -math.pi / 2
    end, _ = simulate((4.02, -1.5, yaw + 0.05), (4.0, -1.1), yaw, reverse=True)
    assert end is not None
    rem, lat = errors(end, (4.0, -1.1), yaw)
    assert rem > -0.01 and abs(lat) < 0.03
    assert abs(wrap(end[2] - yaw)) < math.radians(10)


def test_reverse_never_drives_forward():
    yaw = 0.0
    for lat in (-0.1, 0.0, 0.1):
        v, _, done, _, _ = track_cmd((1.0, lat, 0.2), (0.0, 0.0), yaw, reverse=True)
        assert not done and v < 0
