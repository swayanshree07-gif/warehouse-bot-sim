"""Day 1 smoke test: headless Isaac Sim + ROS 2 bridge publishing /clock.

Builds a trivial stage (ground plane + falling cube), adds an action graph that
publishes simulation time on /clock, plays the timeline for --seconds of wall
time and reports the cube height (proves physics ran).

Run via scripts/smoke_clock.sh (sources env.sh, wraps in timeout).
"""
import argparse
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--seconds", type=float, default=40.0, help="wall time to keep publishing")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 640, "height": 480})

import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics  # noqa: E402,F401

ext_mgr = omni.kit.app.get_app().get_extension_manager()
ext_mgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
app.update()
if not ext_mgr.is_extension_enabled("isaacsim.ros2.bridge"):
    print("SMOKE_FAIL ros2 bridge extension did not enable", flush=True)
    app.close()
    sys.exit(2)

import omni.graph.core as og  # noqa: E402

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)

ground = UsdGeom.Cube.Define(stage, "/World/Ground")
ground.CreateSizeAttr(1.0)
ground.AddTranslateOp().Set(Gf.Vec3f(0, 0, -0.05))
ground.AddScaleOp().Set(Gf.Vec3f(10, 10, 0.1))
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

cube = UsdGeom.Cube.Define(stage, "/World/Box")
cube.CreateSizeAttr(0.2)
cube_xf = cube.AddTranslateOp()
cube_xf.Set(Gf.Vec3f(0, 0, 1.0))
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(1.0)

og.Controller.edit(
    {"graph_path": "/World/ClockGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ],
    },
)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
print("SMOKE_READY publishing /clock", flush=True)

t0 = time.time()
frames = 0
while time.time() - t0 < args.seconds and app.is_running():
    app.update()
    frames += 1

from isaacsim.core.utils.xforms import get_world_pose  # noqa: E402

try:
    pos, _ = get_world_pose("/World/Box")
    z = float(pos[2])
except Exception:  # fall back to raw USD read
    z = float(UsdGeom.Xformable(cube.GetPrim()).ComputeLocalToWorldTransform(0).ExtractTranslation()[2])
sim_time = timeline.get_current_time()
print(f"SMOKE_STATS frames={frames} sim_time={sim_time:.2f} box_z={z:.3f}", flush=True)
ok = z < 0.5  # box fell from 1.0 m onto the ground
print("SMOKE_OK" if ok else "SMOKE_FAIL box did not fall", flush=True)
timeline.stop()
app.close()
sys.exit(0 if ok else 1)
