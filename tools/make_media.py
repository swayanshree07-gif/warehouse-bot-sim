#!/usr/bin/env python3
"""Generate README media from real project runs/logs (visuals pass).

  timeout 300 .venv/bin/python tools/make_media.py
Runs the light 2D fleet simulator (same fleet/traffic code as Isaac mode),
records every tick and renders docs/media/lightsim_fleet.gif, then plots the
lightsim and Isaac fleet reports and the saved SLAM map.
"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "ros_ws", "src", "warehouse_tasks"))
from warehouse_tasks.lightsim import DEFAULT_SCENARIO, LightSim  # noqa: E402
from warehouse_tasks.registry import Registry  # noqa: E402
from warehouse_tasks.traffic import Roadmap, read_pgm  # noqa: E402

BR = os.path.join(ROOT, "ros_ws", "src", "warehouse_bringup")
OUT = os.path.join(ROOT, "docs", "media")
STARTS = {"robot1": (-2.0, 0.0), "robot2": (2.0, 0.0), "robot3": (6.0, 0.0)}
COL = {"robot1": "#2a78d6", "robot2": "#e0662a", "robot3": "#1f9e6e"}
os.makedirs(OUT, exist_ok=True)


def load_map():
    import yaml
    y = yaml.safe_load(open(os.path.join(BR, "maps", "warehouse.yaml")))
    img = np.array(Image.open(os.path.join(BR, "maps", y["image"])))
    h, w = img.shape
    ox, oy = y["origin"][:2]
    r = y["resolution"]
    return img, [ox, ox + w * r, oy, oy + h * r]


def lightsim_gif():
    rm = Roadmap.from_map(os.path.join(BR, "maps", "warehouse.yaml"))
    reg = Registry.load(os.path.join(BR, "config", "registry.yaml"))
    sim = LightSim(rm, reg, STARTS)
    for b, s in DEFAULT_SCENARIO:
        sim.submit(b, s)
    frames = []
    while not sim.finished() and sim.t < 600:
        frames.append((sim.t, {n: sim.pos(n) for n in sim.tm.agents},
                       {n: p[0] for n, p in sim.phase.items()},
                       sum(t.state == "done" for t in sim.fleet.tasks.values()),
                       sim.events[-1][1] if sim.events else ""))
        sim.step()
    frames.append((sim.t, {n: sim.pos(n) for n in sim.tm.agents},
                   {n: p[0] for n, p in sim.phase.items()},
                   sum(t.state == "done" for t in sim.fleet.tasks.values()),
                   "all tasks done"))
    rep = sim.report()
    img, ext = load_map()
    shelves = {k: v for k, v in sim.fleet.registry.shelves.items()}
    pil = []
    trail = {n: [] for n in STARTS}
    fig, ax = plt.subplots(figsize=(6.4, 4.0), dpi=100)
    for t, pos, ph, done, ev in frames:
        ax.clear()
        ax.imshow(img, cmap="gray", extent=ext, origin="upper", vmin=0, vmax=255)
        for name, sh in shelves.items():
            p = getattr(sh, "pose", None) or getattr(sh, "center", None)
            if p is None and hasattr(sh, "x"):
                p = (sh.x, sh.y)
            if p is not None:
                ax.text(p[0], p[1], name, fontsize=7, ha="center", va="center",
                        color="#555", bbox=dict(fc="#fff8", ec="none", pad=1))
        for n, xy in pos.items():
            trail[n].append(xy)
            tr = np.array(trail[n][-15:])
            ax.plot(tr[:, 0], tr[:, 1], "-", color=COL[n], alpha=0.4, lw=2)
            ax.plot(*xy, "o", ms=11, color=COL[n], mec="k")
            ax.text(xy[0], xy[1] + 0.45, f"{n[-1]}:{ph[n]}", fontsize=7,
                    ha="center", color=COL[n], weight="bold")
        ax.set_title(f"lightsim  t={t:4.0f}s  tasks done {done}/{len(DEFAULT_SCENARIO)}"
                     f"\n{ev}", fontsize=9)
        ax.set_xticks([]); ax.set_yticks([])
        fig.tight_layout()
        fig.canvas.draw()
        pil.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3]))
    plt.close(fig)
    pal = [p.quantize(colors=64, method=Image.Quantize.MEDIANCUT) for p in pil]
    pal[0].save(os.path.join(OUT, "lightsim_fleet.gif"), save_all=True,
                append_images=pal[1:] + [pal[-1]] * 10, duration=110, loop=0, optimize=True)
    return rep


def bars(reports, path):
    fig, axes = plt.subplots(1, len(reports), figsize=(9, 3.2), dpi=100)
    for ax, (title, rep) in zip(axes, reports):
        pr = rep["metrics"]["per_robot"]
        names = sorted(pr)
        busy = [pr[n]["busy_s"] - pr[n]["traffic_wait_s"] for n in names]
        wait = [pr[n]["traffic_wait_s"] for n in names]
        idle = [pr[n]["idle_s"] for n in names]
        ax.barh(names, busy, color="#2a78d6", label="busy (moving / pick / drop)")
        ax.barh(names, wait, left=busy, color="#e0a32a", label="traffic wait")
        ax.barh(names, idle, left=np.add(busy, wait), color="#c9ccd1", label="idle")
        for i, n in enumerate(names):
            ax.text(1, i, f" {pr[n]['tasks_done']} tasks", va="center", color="w", fontsize=8)
        m = rep["metrics"]
        ax.set_title(f"{title}\n{m['tasks_per_hour']} tasks/h, idle {m['mean_idle_pct']}%",
                     fontsize=9)
        ax.set_xlabel("seconds"); ax.invert_yaxis()
        ax.spines[["top", "right"]].set_visible(False)
    axes[0].legend(loc="lower center", bbox_to_anchor=(1.1, -0.45), ncol=3, fontsize=8,
                   frameon=False)
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight"); plt.close(fig)


def gantt(rep, path):
    fig, ax = plt.subplots(figsize=(8, 2.8), dpi=100)
    names = sorted(rep["per_robot_tasks"])
    for t in rep["tasks"]:
        if t.get("t_done") is None:
            continue
        i = names.index(t["robot"])
        ax.barh(i, t["t_done"] - t["t_assign"], left=t["t_assign"], color=COL[t["robot"]],
                edgecolor="w")
        ax.text(t["t_assign"] + 4, i, f"{t['box']}→{t['dst'][0]}", va="center",
                fontsize=7, color="w")
    ax.set_yticks(range(len(names)), names); ax.invert_yaxis()
    ax.set_xlabel("sim time (s)")
    ax.set_title(f"Isaac Sim fleet run: {rep['tasks_done']} tasks done, "
                 f"min separation {rep['min_separation_m']} m", fontsize=9)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def maps(path):
    fig, axes = plt.subplots(1, 2, figsize=(8, 3.4), dpi=100)
    for ax, (f, title) in zip(axes, [("warehouse", "packaged map (slam_toolbox)"),
                                     ("check_slam_map", "SLAM re-check run")]):
        img = Image.open(os.path.join(ROOT, "out", "maps", f + ".pgm"))
        ax.imshow(img, cmap="gray"); ax.set_title(title, fontsize=9); ax.axis("off")
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


if __name__ == "__main__":
    rep = lightsim_gif()
    print("lightsim", {k: rep[k] for k in ("tasks_done", "collisions", "min_separation_m",
                                         "deadlocks_resolved")}, rep["metrics"]["tasks_per_hour"])
    isaac = json.load(open(os.path.join(ROOT, "out", "fleet_isaac_report.json")))
    bars([("Light 2D sim (9 tasks)", rep), ("Isaac Sim (6 tasks)", isaac)],
         os.path.join(OUT, "fleet_utilisation.png"))
    gantt(isaac, os.path.join(OUT, "isaac_task_timeline.png"))
    maps(os.path.join(OUT, "slam_maps.png"))
