"""Fleet spawn poses for the Isaac multi-robot scene (Day 19), pure Python.

Robots stand in the central aisle (y=0, shelves at y=+-2), 3 m apart, facing +x.
Namespaces robot1..N; TF frames are prefixed ("robot1/odom", "robot1/base_link").
"""
FLEET_STARTS = {
    "robot1": (-3.0, 0.0, 0.0),
    "robot2": (0.0, 0.0, 0.0),
    "robot3": (3.0, 0.0, 0.0),
}


def prim_path(ns):
    return f"/World/{ns.capitalize()}"


def frames(ns):
    return f"{ns}/odom", f"{ns}/base_link", f"{ns}/laser"


def topics(ns):
    return [f"/{ns}/{t}" for t in ("cmd_vel", "odom", "scan")]
