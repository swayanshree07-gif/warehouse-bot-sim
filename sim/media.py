"""Headless camera capture (Day 26): PNG frames from low-res render products.

Import after SimulationApp exists. Two cameras:
  - "overview": fixed, high in a room corner looking at the whole warehouse;
  - "chase": child of the robot chassis, behind and above it, looking forward.
Each gets a replicator render product (default 480x270) with an "rgb"
annotator. `MediaRecorder.capture(t)` writes <dir>/<cam>/frame_<NNNN>.png
every `every` sim seconds. Rendering only happens when capture is enabled,
so normal runs (no --capture) keep the RTX renderer idle.

Decision: replicator annotators instead of isaacsim.sensors.camera.Camera
- fewer moving parts, works headless, frames come back as numpy arrays.
"""
import os

import numpy as np
from pxr import Gf, UsdGeom

import warehouse_layout as L


def _look_at_xform(eye, target, up=(0.0, 0.0, 1.0)):
    """USD camera transform (camera looks along -Z, +Y up) from eye -> target."""
    eye, target, up = (np.asarray(v, dtype=float) for v in (eye, target, up))
    fwd = target - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    cup = np.cross(right, fwd)
    m = Gf.Matrix4d(1.0)
    # row-vector convention: rows are the camera axes in world coordinates
    m.SetRow(0, Gf.Vec4d(*right, 0.0))
    m.SetRow(1, Gf.Vec4d(*cup, 0.0))
    m.SetRow(2, Gf.Vec4d(*(-fwd), 0.0))
    m.SetRow(3, Gf.Vec4d(*eye, 1.0))
    return m


def _camera(stage, path, matrix, focal=18.0):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    cam.AddTransformOp().Set(matrix)
    return path


def build_cameras(stage, robot_path="/World/Robot"):
    """Create the overview and chase cameras; returns {name: prim path}."""
    hx, hy = L.ROOM_X / 2, L.ROOM_Y / 2
    overview = _camera(stage, "/World/Cameras/overview",
                       _look_at_xform((-hx - 0.5, -hy - 2.0, 9.0), (0.3, 0.3, 0.0)), focal=15.0)
    chase = _camera(stage, f"{robot_path}/chassis/chase_cam",
                    _look_at_xform((-1.4, 0.0, 1.0), (0.8, 0.0, 0.1)), focal=14.0)
    return {"overview": overview, "chase": chase}


class MediaRecorder:
    def __init__(self, stage, out_dir, every=0.5, res=(480, 270), cameras=None):
        import omni.replicator.core as rep

        self.out_dir, self.every, self.res = out_dir, float(every), tuple(res)
        self.cams = cameras or build_cameras(stage)
        self.annots, self.counts, self.next_t = {}, {}, 0.0
        self.blank = 0
        for name, path in self.cams.items():
            rp = rep.create.render_product(path, self.res)
            an = rep.AnnotatorRegistry.get_annotator("rgb")
            an.attach([rp])
            self.annots[name] = an
            self.counts[name] = 0
            os.makedirs(os.path.join(out_dir, name), exist_ok=True)

    def capture(self, t, force=False):
        """Write one frame per camera if `every` sim seconds have passed."""
        if not force and t < self.next_t:
            return 0
        self.next_t = t + self.every
        from PIL import Image

        n = 0
        for name, an in self.annots.items():
            data = an.get_data()
            if data is None or getattr(data, "size", 0) == 0:
                continue
            img = np.asarray(data)[..., :3].astype(np.uint8)
            if img.std() < 1.0:  # renderer not warmed up yet (all black)
                self.blank += 1
                continue
            path = os.path.join(self.out_dir, name, f"frame_{self.counts[name]:04d}.png")
            Image.fromarray(img).save(path)
            self.counts[name] += 1
            n += 1
        return n

    def summary(self):
        return " ".join(f"{k}={v}" for k, v in self.counts.items()) + f" blank={self.blank}"


def warm_up(app, n=20):
    """A few extra frames so the RTX pipeline produces non-black images."""
    for _ in range(n):
        app.update()
