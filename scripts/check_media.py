"""Validate captured frames and videos (Day 26).

  /usr/bin/python3 scripts/check_media.py MEDIA_DIR [--min-frames N]
        [--cameras overview,chase] [--need-video] [--min-change 0.1]

Checks per camera: at least N frames (frame_0000..), all the same size, none
blank (pixel std > 5), and the first and last frame differ: more than
--min-change percent of the pixels changed by > 25 grey levels in some channel
(the robot moved / the view changed; the robot is small in the overview
camera, ~0.3 % for a 1 m drive, while render noise between static frames is
< 0.01 %). With --need-video, <camera>.mp4 and demo.mp4 must exist and ffprobe
must report a duration > 0 and the expected frame size. Prints one line per
check, then MEDIA_PASS or MEDIA_FAIL (exit 1).
"""
import argparse
import glob
import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image


def load(path):
    return np.asarray(Image.open(path).convert("RGB"), dtype=np.float32)


def changed_percent(a, b, level=25.0):
    """Percent of pixels where any channel differs by more than `level`."""
    return float((np.abs(a - b).max(axis=2) > level).mean() * 100.0)


def check_camera(d, name, min_frames, min_change):
    errs, frames = [], sorted(glob.glob(os.path.join(d, name, "frame_*.png")))
    if len(frames) < min_frames:
        return [f"{name}: {len(frames)} frames < {min_frames}"], None
    # names must be contiguous for ffmpeg's frame_%04d pattern
    want = [os.path.join(d, name, f"frame_{i:04d}.png") for i in range(len(frames))]
    if frames != want:
        errs.append(f"{name}: frame numbering has gaps")
    sizes, blank = set(), 0
    for f in frames:
        with Image.open(f) as im:
            sizes.add(im.size)
    # std of every 5th frame (cheap), plus first and last
    for f in frames[::5] + [frames[-1]]:
        if load(f).std() <= 5.0:
            blank += 1
    if len(sizes) != 1:
        errs.append(f"{name}: mixed frame sizes {sorted(sizes)}")
    if blank:
        errs.append(f"{name}: {blank} blank frames")
    change = changed_percent(load(frames[0]), load(frames[-1]))
    if change <= min_change:
        errs.append(f"{name}: only {change:.3f} % of pixels changed first->last (scene static?)")
    size = sorted(sizes)[0]
    print(f"{'ok  ' if not errs else 'FAIL'} {name}: {len(frames)} frames {size[0]}x{size[1]}, "
          f"first->last {change:.2f} % pixels changed")
    return errs, size


def probe(path):
    out = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                          "stream=width,height,codec_name:format=duration", "-of", "json", path],
                         capture_output=True, text=True, timeout=30)
    if out.returncode != 0:
        return None
    j = json.loads(out.stdout)
    s = j["streams"][0]
    return s["codec_name"], int(s["width"]), int(s["height"]), float(j["format"]["duration"])


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--min-frames", type=int, default=10)
    ap.add_argument("--cameras", default="overview,chase")
    ap.add_argument("--need-video", action="store_true")
    ap.add_argument("--min-change", type=float, default=0.1, help="percent of pixels")
    a = ap.parse_args(argv)
    errs, sizes = [], {}
    for cam in a.cameras.split(","):
        e, size = check_camera(a.dir, cam, a.min_frames, a.min_change)
        errs += e
        sizes[cam] = size
    if a.need_video:
        for cam in list(sizes) + ["demo"]:
            path = os.path.join(a.dir, f"{cam}.mp4")
            p = probe(path) if os.path.exists(path) else None
            if p is None:
                errs.append(f"{cam}.mp4 missing or unreadable")
                continue
            codec, w, h, dur = p
            if cam == "demo":
                ok_size = all(sizes.values()) and (w, h) == (sum(s[0] for s in sizes.values()),
                                                             max(s[1] for s in sizes.values()))
            else:
                ok_size = sizes[cam] is not None and (w, h) == sizes[cam]
            good = dur > 0 and ok_size
            print(f"{'ok  ' if good else 'FAIL'} {cam}.mp4: {codec} {w}x{h} {dur:.1f} s, "
                  f"{os.path.getsize(path) / 1e6:.2f} MB")
            if not good:
                errs.append(f"{cam}.mp4 bad (size {w}x{h}, duration {dur})")
    for e in errs:
        print("FAIL", e)
    total = sum(os.path.getsize(f) for f in glob.glob(os.path.join(a.dir, "**", "*"), recursive=True)
                if os.path.isfile(f))
    print(("MEDIA_PASS" if not errs else "MEDIA_FAIL") + f" {a.dir} ({total / 1e6:.1f} MB)")
    return 0 if not errs else 1


if __name__ == "__main__":
    sys.exit(main())
