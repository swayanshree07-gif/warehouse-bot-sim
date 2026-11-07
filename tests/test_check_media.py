"""scripts/check_media.py on synthetic frames (Day 26)."""
import os
import shutil
import subprocess
import sys

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import check_media  # noqa: E402

W, H = 64, 36


def _frames(d, cam, n, moving=True, blank_at=None):
    os.makedirs(os.path.join(d, cam), exist_ok=True)
    rng = np.random.default_rng(0)
    base = rng.integers(60, 200, size=(H, W, 3), dtype=np.uint8)
    for i in range(n):
        img = base.copy()
        x = (i * 4) % (W - 8) if moving else 0
        img[10:18, x:x + 8] = (250, 200, 0)  # the "robot"
        if i == blank_at:
            img[:] = 0
        Image.fromarray(img).save(os.path.join(d, cam, f"frame_{i:04d}.png"))


def test_changed_percent():
    a = np.zeros((10, 10, 3), np.float32)
    b = a.copy()
    b[0, :5, 1] = 100.0
    assert check_media.changed_percent(a, b) == pytest.approx(5.0)
    b[0, 5:, 1] = 10.0  # below the 25-level threshold
    assert check_media.changed_percent(a, b) == pytest.approx(5.0)


def test_pass(tmp_path, capsys):
    _frames(str(tmp_path), "overview", 12)
    _frames(str(tmp_path), "chase", 12)
    assert check_media.main([str(tmp_path)]) == 0
    assert "MEDIA_PASS" in capsys.readouterr().out


def test_static_scene_fails(tmp_path, capsys):
    _frames(str(tmp_path), "overview", 12, moving=False)
    assert check_media.main([str(tmp_path), "--cameras", "overview"]) == 1
    assert "scene static" in capsys.readouterr().out


def test_blank_and_too_few_frames(tmp_path, capsys):
    _frames(str(tmp_path), "overview", 12, blank_at=5)
    _frames(str(tmp_path), "chase", 3)
    assert check_media.main([str(tmp_path)]) == 1
    out = capsys.readouterr().out
    assert "overview: 1 blank frames" in out
    assert "chase: 3 frames < 10" in out


def test_gap_in_numbering(tmp_path, capsys):
    _frames(str(tmp_path), "overview", 12)
    os.remove(os.path.join(str(tmp_path), "overview", "frame_0004.png"))
    assert check_media.main([str(tmp_path), "--cameras", "overview"]) == 1
    assert "gaps" in capsys.readouterr().out


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg not installed")
def test_make_video_and_probe(tmp_path, capsys):
    _frames(str(tmp_path), "overview", 12)
    _frames(str(tmp_path), "chase", 12)
    script = os.path.join(os.path.dirname(__file__), "..", "scripts", "make_video.sh")
    r = subprocess.run(["bash", script, str(tmp_path)], capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stdout + r.stderr
    assert check_media.main([str(tmp_path), "--need-video"]) == 0
    out = capsys.readouterr().out
    assert f"demo.mp4: h264 {2 * W}x{H}" in out
