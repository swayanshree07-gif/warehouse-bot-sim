"""Headless warehouse with 3 namespaced robots (Day 19).

Each robot robotN gets /robotN/cmd_vel, /robotN/odom, /robotN/scan,
/robotN/joint_states and TF robotN/odom->robotN/base_link (+ static
robotN/base_link->robotN/laser) on the shared /tf, /tf_static. One /clock.
Prints SIM_READY, then `FPOSE t= robot1=x,y,yaw robot2=... robot3=...` every
--pose-every sim s, and `VRAM_MB=<n>` (nvidia-smi, whole GPU) every 5 s.
Day 22: each robot also gets a namespaced lift + virtual gripper
(/robotN/lift/command, /robotN/lift/height, /robotN/gripper/{attach,detach,held})
and one global /boxes (BoxStates with holder = robot namespace). Prints
`FBOXES t= ...` every --box-every sim s. --starts overrides spawn poses:
  --starts robot1=x,y,yaw robot2=... (world frame, yaw in degrees)

  python sim/run_fleet_sim.py --seconds 90
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

ap = argparse.ArgumentParser()
ap.add_argument("--seconds", type=float, default=90.0)
ap.add_argument("--pose-every", type=float, default=0.5)
ap.add_argument("--box-every", type=float, default=1.0)
ap.add_argument("--capture", default="", help="write overview + robot1 chase PNG frames here (Day 26)")
ap.add_argument("--capture-every", type=float, default=0.5, help="sim seconds between frames")
ap.add_argument("--capture-res", default="480x270", help="WxH of each camera")
ap.add_argument("--starts", nargs="*", default=[], help="ns=x,y,yaw_deg overrides")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 640, "height": 480})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402

from fleet_layout import frames, parse_starts, prim_path  # noqa: E402
from robot import build_robot  # noqa: E402
from ros_graphs import (  # noqa: E402
    build_clock_graph, build_cmd_vel_graph, build_odom_graph, build_scan_graph, enable_ros_extensions)
from scene import build_warehouse  # noqa: E402
from sensors import build_laser  # noqa: E402

enable_ros_extensions(app)
stage = omni.usd.get_context().get_stage()
build_warehouse(stage)
build_clock_graph()
STARTS = parse_starts(args.starts)
for ns, (x, y, yaw) in STARTS.items():
    root = prim_path(ns)
    odom_f, base_f, laser_f = frames(ns)
    build_robot(stage, path=root, x=x, y=y, yaw=math.radians(yaw))
    g = f"/World/ROS/{ns}"
    build_cmd_vel_graph(robot_path=root, topic=f"{ns}/cmd_vel", path=f"{g}/CmdVel")
    build_odom_graph(robot_path=root, path=f"{g}/Odom", odom_frame=odom_f, base_frame=base_f, ns=ns)
    build_scan_graph(build_laser(robot_path=root), path=f"{g}/Scan", frame=laser_f, base_frame=base_f, ns=ns)

from isaacsim.core.utils.xforms import get_world_pose  # noqa: E402
from box_handler import BoxHandler  # noqa: E402

peers = []
handlers = [BoxHandler(stage, robot=prim_path(ns), ns=ns, peers=peers, owns_boxes=(i == 0))
            for i, ns in enumerate(STARTS)]

recorder = None
if args.capture:
    from media import MediaRecorder, build_cameras, warm_up  # noqa: E402

    w, h = (int(v) for v in args.capture_res.lower().split("x"))
    recorder = MediaRecorder(stage, args.capture, every=args.capture_every, res=(w, h),
                             cameras=build_cameras(stage, robot_path=prim_path(next(iter(STARTS)))))


def pose(ns):
    p, q = get_world_pose(f"{prim_path(ns)}/chassis")
    w, x, y, z = (float(v) for v in q)
    return float(p[0]), float(p[1]), math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def vram_mb():
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=5).stdout
        return int(out.split()[0])
    except Exception:
        return -1


_stop = {"sig": None}
signal.signal(signal.SIGINT, lambda s, f: _stop.update(sig=s))
signal.signal(signal.SIGTERM, lambda s, f: _stop.update(sig=s))

tl = omni.timeline.get_timeline_interface()
tl.play()
app.update()
if recorder is not None:
    warm_up(app)
    print(f"CAPTURE_ON dir={args.capture} every={args.capture_every} res={args.capture_res}", flush=True)
print(f"VRAM_MB={vram_mb()}", flush=True)
print("SIM_READY", flush=True)
wall0, sim0, next_pose, next_vram = time.time(), tl.get_current_time(), 0.0, 0.0
next_box, next_pub = 0.0, 0.0
while app.is_running() and time.time() - wall0 < args.seconds and _stop["sig"] is None:
    for h in handlers:
        for _ in range(4):
            h.spin()
        for ev in h.events:
            print(f"{ev} ns={h.ns} t={tl.get_current_time() - sim0:.2f}", flush=True)
        h.events.clear()
    app.update()
    st = tl.get_current_time() - sim0
    if st >= next_pub:
        for h in handlers:
            h.publish()
        next_pub += 0.1
    if st >= next_box:
        print(handlers[0].fleet_status_line(st), flush=True)
        next_box += args.box_every
    if st >= next_pose:
        parts = " ".join("{}={:.3f},{:.3f},{:.1f}".format(ns, *pose(ns)) for ns in STARTS)
        print(f"FPOSE t={st:.2f} {parts}", flush=True)
        next_pose += args.pose_every
    if time.time() - wall0 >= next_vram:
        print(f"VRAM_MB={vram_mb()}", flush=True)
        next_vram += 5.0
    if recorder is not None:
        recorder.capture(st)
    ahead = st - (time.time() - wall0)
    if ahead > 0:
        time.sleep(ahead)
tl.stop()
for h in reversed(handlers):   # the /boxes owner (index 0) shuts rclpy down last
    h.shutdown()
if recorder is not None:
    print(f"CAPTURE_DONE {recorder.summary()}", flush=True)
print(f"SIM_DONE reason={'signal' if _stop['sig'] else 'time'}", flush=True)
app.close()
