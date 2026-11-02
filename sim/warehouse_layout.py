"""Warehouse layout: single source of truth for scene geometry (pure Python).

Used by the scene builder (sim/scene.py), and later by map checks and the
shelf/box registry. Units: metres, Z up, origin at the room centre.

Decision (Day 2): shelves are solid low racks (lidar-visible, simple collision
boxes). Boxes sit in slots on the rack top; the robot will lift/attach them.
"""
from dataclasses import dataclass, field

ROOM_X = 10.0          # interior length (x)
ROOM_Y = 8.0           # interior width (y)
WALL_T = 0.1           # wall thickness
WALL_H = 1.0           # wall height

SHELF_SIZE = (1.5, 0.4, 0.35)   # x, y, z of a rack block
BOX_SIZE = 0.2                  # cube edge
BOX_MASS = 0.5                  # kg
SLOTS_PER_SHELF = 3


@dataclass
class Shelf:
    name: str
    x: float
    y: float
    yaw_deg: float = 0.0   # 0 = long side along x
    slots: list = field(default_factory=list)  # [(x, y, z_top)]


@dataclass
class Box:
    name: str
    shelf: str
    slot: int
    x: float
    y: float
    z: float   # centre height


def _make_shelves():
    # Four racks: two rows, leaving a 2 m+ wide central aisle and side aisles.
    spec = [
        ("shelf_A", -2.5, 2.0),
        ("shelf_B", 2.5, 2.0),
        ("shelf_C", -2.5, -2.0),
        ("shelf_D", 2.5, -2.0),
    ]
    shelves = []
    sx, _, sz = SHELF_SIZE
    pitch = sx / SLOTS_PER_SHELF
    for name, x, y in spec:
        slots = [(x - sx / 2 + pitch * (i + 0.5), y, sz) for i in range(SLOTS_PER_SHELF)]
        shelves.append(Shelf(name, x, y, 0.0, slots))
    return shelves


SHELVES = _make_shelves()


def _make_boxes(n=10):
    boxes = []
    i = 0
    for shelf in SHELVES:
        for k, (x, y, ztop) in enumerate(shelf.slots):
            if i >= n:
                return boxes
            boxes.append(Box(f"box_{i:02d}", shelf.name, k, x, y, ztop + BOX_SIZE / 2))
            i += 1
    return boxes


BOXES = _make_boxes(10)


def walls():
    """Return wall boxes as (name, centre(x,y,z), size(x,y,z))."""
    hx, hy, t, h = ROOM_X / 2, ROOM_Y / 2, WALL_T, WALL_H
    return [
        ("wall_north", (0.0, hy + t / 2, h / 2), (ROOM_X + 2 * t, t, h)),
        ("wall_south", (0.0, -hy - t / 2, h / 2), (ROOM_X + 2 * t, t, h)),
        ("wall_east", (hx + t / 2, 0.0, h / 2), (t, ROOM_Y, h)),
        ("wall_west", (-hx - t / 2, 0.0, h / 2), (t, ROOM_Y, h)),
    ]
