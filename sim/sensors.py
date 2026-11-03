"""Robot sensors (Day 6): planar 2D lidar from a PhysX raycast sensor.

Import after SimulationApp exists and after enable_ros_extensions() (which also
enables isaacsim.sensors.experimental.physics + isaacsim.sensors.physics.nodes).
The sensor prim is a child of the chassis rigid body, so it moves with it.
Rays are cast by PhysX (no RTX rendering), which keeps VRAM use low.
"""
import math

import robot_params as P


def build_laser(robot_path="/World/Robot", chassis="chassis", name="laser"):
    """Create the lidar under <robot>/<chassis>; returns the sensor prim path."""
    from isaacsim.sensors.experimental.physics import Raycast

    dirs = [[math.cos(math.radians(a)), math.sin(math.radians(a)), 0.0] for a in P.laser_angles_deg()]
    rc = Raycast.create(
        f"{robot_path}/{chassis}/{name}",
        min_range=P.LASER_RANGE[0],
        max_range=P.LASER_RANGE[1],
        ray_origins=[[0.0, 0.0, 0.0]] * len(dirs),
        ray_directions=dirs,
        translations=[list(P.LASER_POS)],
    )
    return rc.paths[0]
