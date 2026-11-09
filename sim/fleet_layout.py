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


# The saved map (Day 11) was built from the single-robot start at world (-2, 0, 0),
# so map frame = world frame shifted by +2 m in x (no rotation).
MAP_ORIGIN_WORLD = (-2.0, 0.0)


def world_to_map(x, y):
    return x - MAP_ORIGIN_WORLD[0], y - MAP_ORIGIN_WORLD[1]


def map_starts():
    """AMCL initial poses (x, y, yaw) in the map frame, per namespace."""
    return {ns: (*world_to_map(x, y), yaw) for ns, (x, y, yaw) in FLEET_STARTS.items()}


def parse_starts(items, base=None):
    """["robot2=1,2,90", ...] -> FLEET_STARTS copy with those poses replaced (Day 22)."""
    out = dict(base if base is not None else FLEET_STARTS)
    for it in items:
        ns, v = it.split("=", 1)
        if ns not in out:
            raise ValueError(f"unknown robot namespace {ns!r}")
        x, y, yaw = (float(c) for c in v.split(","))
        out[ns] = (x, y, yaw)
    return out
