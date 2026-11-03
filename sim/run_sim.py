"""Headless warehouse sim with the robot and ROS 2 bridge (Days 4+).

Publishes /clock, /odom, TF odom->base_link, /joint_states, /scan + static TF
base_link->laser; subscribes /cmd_vel. Paced to real time by default (the
headless sim otherwise runs ~2.5x faster). Prints `SIM_READY` once playing and
`POSE t=<sim s> x= y= yaw=<deg>` every --pose-every seconds (ground truth of the
chassis) so scripted checks can read the robot state from the log.

  python sim/run_sim.py --seconds 60 [--no-realtime] [--start=-2,0,0]
"""
import argparse
import math
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

ap = argparse.ArgumentParser()
ap.add_argument("--seconds", type=float, default=60.0, help="wall-clock run time")
ap.add_argument("--no-realtime", action="store_true")
ap.add_argument("--pose-every", type=float, default=0.25)
ap.add_argument("--start", default="-2,0,0", help="x,y,yaw_deg")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 640, "height": 480})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402

from robot import build_robot  # noqa: E402
from ros_graphs import (  # noqa: E402
    build_clock_graph, build_cmd_vel_graph, build_odom_graph, build_scan_graph, enable_ros_extensions)
from scene import build_warehouse  # noqa: E402
from sensors import build_laser  # noqa: E402

enable_ros_extensions(app)
stage = omni.usd.get_context().get_stage()
build_warehouse(stage)
sx, sy, syaw = (float(v) for v in args.start.split(","))
build_robot(stage, x=sx, y=sy, yaw=math.radians(syaw))
build_clock_graph()
build_cmd_vel_graph()
build_odom_graph()
build_scan_graph(build_laser())

from isaacsim.core.utils.xforms import get_world_pose  # noqa: E402


def pose():
    p, q = get_world_pose("/World/Robot/chassis")
    w, x, y, z = (float(v) for v in q)
    return float(p[0]), float(p[1]), math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


# Graceful stop on SIGINT/SIGTERM (scripts/run_sim.sh forwards Ctrl-C): leave
# the loop, stop the timeline and close Kit instead of dying mid-frame.
_stop = {"sig": None}


def _on_signal(signum, _frame):
    _stop["sig"] = signum


signal.signal(signal.SIGINT, _on_signal)
signal.signal(signal.SIGTERM, _on_signal)

tl = omni.timeline.get_timeline_interface()
tl.play()
app.update()
print("SIM_READY", flush=True)
wall0, sim0, next_pose = time.time(), tl.get_current_time(), 0.0
while app.is_running() and time.time() - wall0 < args.seconds and _stop["sig"] is None:
    app.update()
    st = tl.get_current_time() - sim0
    if st >= next_pose:
        x, y, yaw = pose()
        print(f"POSE t={st:.2f} x={x:.3f} y={y:.3f} yaw={yaw:.1f}", flush=True)
        next_pose += args.pose_every
    if not args.no_realtime:
        ahead = st - (time.time() - wall0)
        if ahead > 0:
            time.sleep(ahead)
tl.stop()
print(f"SIM_DONE reason={'signal' if _stop['sig'] else 'time'}", flush=True)
app.close()
