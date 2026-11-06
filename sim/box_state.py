"""Ground-truth box -> shelf slot classification (Day 23). Pure Python.

Used by sim/box_handler.py to fill warehouse_interfaces/BoxStates on /boxes
and by tests. Positions are box centres in the *map* frame (registry frame).

A box is "on" a slot if it is not held, its centre is within XY_TOL of the slot
centre and within Z_TOL of the slot's resting height. Otherwise shelf="" and
slot=-1 (held, on the floor, lying between slots, fell over ...).
"""
import math

XY_TOL = 0.10   # m; the gripper tolerance, so any box the robot could grip counts
Z_TOL = 0.02    # m; a lifted (held) box is 3 cm up, a resting one ~0 mm


def slot_table(registry):
    """[(shelf, index, (x, y, z))] from a registry dict (registry_gen.build/load)."""
    out = []
    for name, sh in registry["shelves"].items():
        for s in sh["slots"]:
            out.append((name, int(s["index"]), tuple(float(v) for v in s["center"])))
    return out


def classify(pos, table, held=False, xy_tol=XY_TOL, z_tol=Z_TOL):
    """(shelf, slot) for a box centre `pos` (map frame), or ("", -1)."""
    if held:
        return "", -1
    best, best_d = ("", -1), math.inf
    for shelf, idx, c in table:
        d = math.hypot(pos[0] - c[0], pos[1] - c[1])
        if d < best_d and d <= xy_tol and abs(pos[2] - c[2]) <= z_tol:
            best, best_d = (shelf, idx), d
    return best


def world_to_map_fn(origin):
    """origin = registry map_origin_in_world [x, y, yaw_deg] (yaw must be 0)."""
    ox, oy = float(origin[0]), float(origin[1])
    assert abs(float(origin[2])) < 1e-9, "map frame rotated w.r.t. world: not supported"
    return lambda p: (p[0] - ox, p[1] - oy, p[2])
