"""Day 3 check: diff-drive robot drives straight, turns, lifts; never flips.

Usage (via scripts/check_robot.sh): python sim/check_robot.py
Builds warehouse + robot headless, saves out/warehouse_robot.usd, then
  1. settle 0.5 s         -> chassis at rest height, upright
  2. v=0.5 m/s for 2 s    -> ~1.0 m forward (+x), small lateral drift
  3. stop 0.5 s, then w=1 rad/s for pi/2 s -> yaw ~ +90 deg, little translation
  4. lift target 0.20 m   -> lift plate rises ~0.20 m
Prints ROBOT_* lines; exit 0 = PASS.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
ROOT = os.path.dirname(HERE)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 640, "height": 480})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402

import robot_params as P  # noqa: E402
from robot import build_robot, set_lift_height, set_wheel_speeds  # noqa: E402
from scene import build_warehouse  # noqa: E402

HZ = 60.0
START = (-2.0, 0.0)   # centre aisle, free for > 4 m along +x

stage = omni.usd.get_context().get_stage()
build_warehouse(stage)
build_robot(stage, x=START[0], y=START[1], yaw=0.0)
os.makedirs(os.path.join(ROOT, "out"), exist_ok=True)
stage.GetRootLayer().Export(os.path.join(ROOT, "out", "warehouse_robot.usd"))

from isaacsim.core.utils.xforms import get_world_pose  # noqa: E402


def pose(prim="/World/Robot/chassis"):
    p, q = get_world_pose(prim)   # q = (w, x, y, z)
    w, x, y, z = (float(v) for v in q)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return float(p[0]), float(p[1]), float(p[2]), roll, pitch, yaw


tl = omni.timeline.get_timeline_interface()
tl.play()
max_tilt = 0.0


TRACE = bool(os.environ.get("ROBOT_TRACE"))


def run(seconds):
    global max_tilt
    for i in range(int(round(seconds * HZ))):
        app.update()
        x, y, z, r, pt, yw = pose()
        max_tilt = max(max_tilt, abs(r), abs(pt))
        if TRACE and i % 10 == 0:
            lw, rw = pose("/World/Robot/left_wheel"), pose("/World/Robot/right_wheel")
            print(f"TRACE t={tl.get_current_time():.2f} x={x:.3f} y={y:.3f} z={z:.3f} r={math.degrees(r):.1f} "
                  f"p={math.degrees(pt):.1f} yaw={math.degrees(yw):.1f} lwz={lw[2]:.3f} rwz={rw[2]:.3f} "
                  f"lw=({lw[0]:.2f},{lw[1]:.2f}) rw=({rw[0]:.2f},{rw[1]:.2f})", flush=True)


results = {}
run(0.5)
x0, y0, z0, *_ , yaw0 = pose()
results["settle"] = abs(z0 - P.chassis_centre_z()) < 0.02
print(f"ROBOT_SETTLE x={x0:.3f} y={y0:.3f} z={z0:.3f} (expect {P.chassis_centre_z():.3f})", flush=True)

v = 0.5
set_wheel_speeds(stage, *P.wheel_speeds(v, 0.0))
t_start = tl.get_current_time()
run(2.0)
dt = tl.get_current_time() - t_start
set_wheel_speeds(stage, 0.0, 0.0)
run(0.5)
x1, y1, z1, *_, yaw1 = pose()
fwd, lat = x1 - x0, y1 - y0
results["forward"] = 0.85 <= fwd <= 1.15 and abs(lat) < 0.10 and abs(yaw1 - yaw0) < math.radians(5)
print(f"ROBOT_FORWARD cmd={v}m/s for {dt:.2f}s sim -> dx={fwd:.3f} dy={lat:.3f} "
      f"dyaw={math.degrees(yaw1 - yaw0):.1f}deg", flush=True)

w = 1.0
set_wheel_speeds(stage, *P.wheel_speeds(0.0, w))
run(math.pi / 2)
set_wheel_speeds(stage, 0.0, 0.0)
run(0.5)
x2, y2, z2, *_, yaw2 = pose()
dyaw = math.degrees(math.atan2(math.sin(yaw2 - yaw1), math.cos(yaw2 - yaw1)))
drift = math.hypot(x2 - x1, y2 - y1)
results["turn"] = 70.0 <= dyaw <= 110.0 and drift < 0.10
print(f"ROBOT_TURN cmd={w}rad/s for {math.pi / 2:.2f}s -> dyaw={dyaw:.1f}deg drift={drift:.3f}m", flush=True)

lz0 = pose("/World/Robot/lift")[2] - z2
set_lift_height(stage, 0.20)
run(1.5)
lz1 = pose("/World/Robot/lift")[2] - pose()[2]
results["lift"] = abs((lz1 - lz0) - 0.20) < 0.02
print(f"ROBOT_LIFT rise={lz1 - lz0:.3f}m (target 0.200)", flush=True)

results["upright"] = max_tilt < math.radians(10)
print(f"ROBOT_TILT max={math.degrees(max_tilt):.2f}deg", flush=True)
tl.stop()
ok = all(results.values())
print(f"ROBOT_RESULTS {results}", flush=True)
print("ROBOT_OK" if ok else "ROBOT_FAIL", flush=True)
app.close()
sys.exit(0 if ok else 1)
