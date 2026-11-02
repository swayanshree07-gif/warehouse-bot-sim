"""Day 2 check: build the warehouse headless, save it, simulate, verify boxes rest.

Usage (via scripts/check_scene.sh): python sim/check_scene.py [--frames 200]
Writes out/warehouse.usd. Exit 0 = PASS.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
ROOT = os.path.dirname(HERE)

ap = argparse.ArgumentParser()
ap.add_argument("--frames", type=int, default=200)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 640, "height": 480})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import UsdGeom  # noqa: E402

import warehouse_layout as L  # noqa: E402
from scene import build_warehouse  # noqa: E402

ctx = omni.usd.get_context()
stage = ctx.get_stage()
counts = build_warehouse(stage)
os.makedirs(os.path.join(ROOT, "out"), exist_ok=True)
usd_path = os.path.join(ROOT, "out", "warehouse.usd")
stage.GetRootLayer().Export(usd_path)
print(f"SCENE_COUNTS {counts} saved={usd_path}", flush=True)

# reload from disk to prove the saved file is self-contained
ctx.open_stage(usd_path)
for _ in range(5):
    app.update()
stage = ctx.get_stage()


def box_pos(name):
    prim = stage.GetPrimAtPath(f"/World/Boxes/{name}")
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()


start = {b.name: box_pos(b.name) for b in L.BOXES}

# Probe: a temporary cube dropped in the free aisle proves physics is stepping
# (added after saving, so it is not part of out/warehouse.usd).
from pxr import Gf, UsdPhysics  # noqa: E402
probe = UsdGeom.Cube.Define(stage, "/World/Probe")
probe.CreateSizeAttr(0.2)
probe.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.0))
UsdPhysics.CollisionAPI.Apply(probe.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(probe.GetPrim())
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(args.frames):
    app.update()

# read the simulated pose (PhysX writes back to USD by default)
from isaacsim.core.utils.xforms import get_world_pose  # noqa: E402

worst = 0.0
for b in L.BOXES:
    p, _ = get_world_pose(f"/World/Boxes/{b.name}")
    d = max(abs(p[i] - start[b.name][i]) for i in range(3))
    worst = max(worst, d)
probe_z = float(get_world_pose("/World/Probe")[0][2])
print(f"SCENE_STATS probe_z={probe_z:.3f} frames={args.frames} sim_time={tl.get_current_time():.2f} "
      f"max_box_drift={worst * 100:.2f}cm", flush=True)
tl.stop()
ok = counts == {"walls": 4, "shelves": 4, "boxes": 10} and worst < 0.01 and 0.05 < probe_z < 0.2
print("SCENE_OK" if ok else "SCENE_FAIL", flush=True)
app.close()
sys.exit(0 if ok else 1)
