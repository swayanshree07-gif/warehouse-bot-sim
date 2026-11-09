"""fleet_manager ROS 2 node (Day 23).

Takes pick/drop tasks over ROS 2, assigns each to the nearest free robot
(`FleetManager`), runs them on a backend and publishes the fleet state.

Backend "light" (the only one so far): the pure-Python `LightSim` stepped in
real time by a timer (one traffic tick every `tick_period` wall seconds), so
the node is testable with no GPU. Day 24 adds an Isaac backend that gates
each robot's next path segment instead.

Interface (decision: plain std_msgs/String so no new interface package
rebuild is needed; a typed FleetTask srv can come later if useful):
  /fleet/tasks   (sub, String)  "box_id shelf_id"  e.g. "box_00 zone_E"
  /fleet/status  (pub, String, 2 Hz + on change) JSON:
      {"t", "robots": {name: {"state","task","phase","xy"}},
       "tasks": {id: {"box","dst","slot","state","robot"}},
       "pending", "rejected", "done", "failed", "collisions",
       "min_separation_m", "metrics"}
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .lightsim import LightSim
from .metrics import compute_metrics
from .registry import Registry
from .traffic import Roadmap

DEFAULT_STARTS = "robot1=-2,0;robot2=2,0;robot3=6,0"


def parse_starts(s):
    out = {}
    for part in filter(None, (p.strip() for p in s.split(";"))):
        name, xy = part.split("=")
        x, y = (float(v) for v in xy.split(","))
        out[name.strip()] = (x, y)
    return out


def parse_task(text):
    """'box shelf' -> (box, shelf). Raises ValueError."""
    parts = text.split()
    if len(parts) != 2:
        raise ValueError(f"expected 'box_id shelf_id', got {text!r}")
    return parts[0], parts[1]


def status_dict(sim):
    tasks = {}
    for t in sim.fleet.tasks.values():
        tasks[str(t.id)] = {"box": t.box_id, "dst": t.plan["dst_shelf"],
                            "slot": t.plan["dst_slot"], "state": t.state,
                            "robot": t.robot}
    robots = {}
    for n, r in sim.fleet.robots.items():
        x, y = sim.pos(n)
        robots[n] = {"state": r.state, "task": r.task_id,
                     "phase": sim.phase[n][0], "xy": [round(x, 2), round(y, 2)]}
    st = sim.fleet.status()
    return {
        "t": sim.t, "robots": robots, "tasks": tasks,
        "pending": [f"{b} {s}" for b, s in sim.pending],
        "rejected": [f"{b} {s}: {e}" for b, s, e in sim.rejected],
        "done": st["done"], "failed": st["failed"],
        "collisions": sim.collisions,
        "min_separation_m": None if math.isinf(sim.min_sep) else round(sim.min_sep, 3),
        "metrics": compute_metrics(list(sim.fleet.tasks.values()), sim.robot_time, sim.t)
        if sim.t > 0 else {},
    }


class FleetManagerNode(Node):
    def __init__(self):
        super().__init__("fleet_manager")
        self.declare_parameter("backend", "light")
        self.declare_parameter("tick_period", 0.05)   # wall s per 1 s sim tick
        self.declare_parameter("starts", DEFAULT_STARTS)
        self.declare_parameter("registry", "")
        self.declare_parameter("map", "")
        backend = self.get_parameter("backend").value
        if backend != "light":
            raise RuntimeError(f"backend {backend!r} not supported yet (only 'light')")
        reg_path, map_path = self.get_parameter("registry").value, self.get_parameter("map").value
        if not reg_path or not map_path:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("warehouse_bringup")
            reg_path = reg_path or os.path.join(share, "config", "registry.yaml")
            map_path = map_path or os.path.join(share, "maps", "warehouse.yaml")
        starts = parse_starts(self.get_parameter("starts").value)
        self.sim = LightSim(Roadmap.from_map(map_path), Registry.load(reg_path), starts)
        self.n_events = 0
        self.pub = self.create_publisher(String, "/fleet/status", 10)
        self.create_subscription(String, "/fleet/tasks", self.on_task, 10)
        self.create_timer(float(self.get_parameter("tick_period").value), self.on_tick)
        self.create_timer(0.5, self.publish)
        self.get_logger().info(f"fleet_manager ready: backend=light robots={sorted(starts)}")

    def on_task(self, msg):
        try:
            box, shelf = parse_task(msg.data)
        except ValueError as e:
            self.sim.rejected.append((msg.data, "", str(e)))
            self.get_logger().warn(str(e))
            return
        self.sim.submit(box, shelf)
        self.get_logger().info(f"task received: {box} -> {shelf}")

    def on_tick(self):
        busy = self.sim.pending or self.sim.fleet.queue or any(
            ph != "idle" for ph, _, _ in self.sim.phase.values())
        if not busy:
            return          # sim time only advances while there is work
        self.sim.step()
        new = self.sim.events[self.n_events:]
        self.n_events = len(self.sim.events)
        for t, m in new:
            self.get_logger().info(f"[{t:6.1f}] {m}")
        if new:
            self.publish()

    def publish(self):
        self.pub.publish(String(data=json.dumps(status_dict(self.sim))))


def main():
    rclpy.init()
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
