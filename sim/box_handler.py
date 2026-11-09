"""Lift command + virtual box gripper over ROS 2 (Days 16-17).

Runs inside the sim process as a plain rclpy node (system Jazzy rclpy works in
the Isaac Sim 6.2 venv, both are Python 3.12), spun without blocking once per
frame from sim/run_sim.py. Import after SimulationApp exists.

  /lift/command   std_msgs/Float64 (sub)  lift target height in m, clamped to
                                         robot_params.LIFT_TRAVEL
  /lift/height    std_msgs/Float64 (pub)  measured lift height, ~10 Hz
  /gripper/attach std_srvs/Trigger        attach the box whose centre is within
                                         GRIP_TOL (xy) of the grip point
                                         GRIP_REACH ahead of base_link
  /gripper/detach std_srvs/Trigger        release the held box
  /gripper/held   std_msgs/String (pub)   held box id or "" (~10 Hz)
  /boxes          warehouse_interfaces/BoxStates (pub, ~10 Hz, Day 23):
                  ground truth of every box (map frame, shelf/slot from
                  sim/box_state.py, held flag)
  /sim/remove_box std_msgs/String (sub)   test hook (Day 22): teleport that box
                  onto the floor in the SE corner, i.e. it goes "missing"

Attach creates a UsdPhysics.FixedJoint lift <-> box at the *current* relative
pose (no snapping), so the box rides on the lift carriage; detach removes the
joint and the box is a free rigid body again. Decision (Day 5 session): no
custom interfaces yet (they come with warehouse_interfaces on Day 18); Trigger
reports the box id in its message.
"""
import math

import rclpy
import rclpy.parameter
from rclpy.signals import SignalHandlerOptions
from pxr import Gf, UsdPhysics
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

import box_state
import registry_gen
import robot_params as P
import warehouse_layout as L
from geometry_msgs.msg import Point
from warehouse_interfaces.msg import BoxState, BoxStates
from isaacsim.core.utils.xforms import get_world_pose
from robot import set_lift_height

GRIP_JOINT = "/World/GripJoint"
# where /sim/remove_box puts boxes (world frame, floor of the SE corner, off all routes)
REMOVED_AT = (4.6, -3.6)


def _pose(path):
    p, q = get_world_pose(path)
    return Gf.Vec3d(*(float(v) for v in p)), Gf.Quatd(*(float(v) for v in q))   # q = (w, x, y, z)


def _yaw(q):
    w, x, y, z = q.GetReal(), *q.GetImaginary()
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class BoxHandler:
    def __init__(self, stage, robot="/World/Robot", boxes_root="/World/Boxes", ns="", peers=None,
                 owns_boxes=True):
        """ns: robot namespace (Day 22 fleet, "" = single robot). peers: list shared by
        every handler in the sim (self is appended) so attach never grabs a box another
        robot holds and /boxes reports the holder. Only the owns_boxes handler
        publishes /boxes and serves /sim/remove_box."""
        self.stage = stage
        self.ns = ns.strip("/")
        self.peers = peers if peers is not None else []
        self.peers.append(self)
        self.owns_boxes = owns_boxes
        self.joint = GRIP_JOINT + (f"_{self.ns}" if self.ns else "")
        self.robot = robot
        self.chassis = f"{robot}/chassis"
        self.lift = f"{robot}/lift"
        self.boxes = {b.name: f"{boxes_root}/{b.name}" for b in L.BOXES}
        self.held = ""
        self.target = 0.0
        self.events = []   # log lines for run_sim to print
        if not rclpy.ok():
            # run_sim.py owns SIGINT/SIGTERM (graceful stop); keep rclpy out of it
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        # sim time (the OmniGraph /clock) for the /boxes header stamps
        self.node = rclpy.create_node("sim_box_handler", namespace=self.ns, parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])
        self.node.create_subscription(Float64, "lift/command", self._on_lift, 10)
        self.pub_h = self.node.create_publisher(Float64, "lift/height", 10)
        self.pub_held = self.node.create_publisher(String, "gripper/held", 10)
        self.node.create_service(Trigger, "gripper/attach", self._on_attach)
        self.node.create_service(Trigger, "gripper/detach", self._on_detach)
        self.pub_boxes = None
        if owns_boxes:
            pre = "/" if self.ns else ""   # fleet: one global /boxes, /sim/remove_box
            self.pub_boxes = self.node.create_publisher(BoxStates, f"{pre}boxes", 10)
            self.node.create_subscription(String, f"{pre}sim/remove_box", self._on_remove, 10)
        reg = registry_gen.build()
        self.slots = box_state.slot_table(reg)
        self.to_map = box_state.world_to_map_fn(reg["map_origin_in_world"])
        self.removed = 0
        self._rest_dz = (P.chassis_top_z() + P.LIFT_SIZE[2] / 2) - P.chassis_centre_z()

    # --- state ---------------------------------------------------------------
    def lift_height(self):
        pc, qc = _pose(self.chassis)
        pl, _ = _pose(self.lift)
        # lift offset along the chassis z axis, minus the rest offset
        up = Gf.Rotation(qc).TransformDir(Gf.Vec3d(0, 0, 1))
        return float(Gf.Dot(pl - pc, up)) - self._rest_dz

    def grip_point(self):
        pc, qc = _pose(self.chassis)
        fwd = Gf.Rotation(qc).TransformDir(Gf.Vec3d(1, 0, 0))
        return pc + fwd * P.GRIP_REACH

    def box_positions(self):
        return {n: _pose(p)[0] for n, p in self.boxes.items()}

    # --- ROS callbacks -------------------------------------------------------
    def _on_lift(self, msg):
        lo, hi = P.LIFT_TRAVEL
        self.target = min(max(float(msg.data), lo), hi)
        set_lift_height(self.stage, self.target, self.robot)
        self.events.append(f"LIFT_CMD target={self.target:.3f}")

    def _on_attach(self, _req, res):
        if self.held:
            res.success, res.message = False, f"already holding {self.held}"
        else:
            g = self.grip_point()
            best, best_d = None, math.inf
            taken = {h.held for h in self.peers if h is not self and h.held}
            for name, p in self.box_positions().items():
                if name in taken:
                    continue
                d = math.hypot(p[0] - g[0], p[1] - g[1])
                if d < best_d and abs(p[2] - g[2]) < 0.6:
                    best, best_d = name, d
            if best is None or best_d > P.GRIP_TOL:
                res.success = False
                res.message = f"no box within {P.GRIP_TOL} m of grip point (nearest {best} at {best_d:.3f} m)"
            else:
                self._make_joint(best)
                self.held = best
                res.success, res.message = True, best
        self.events.append(f"GRIP_ATTACH ok={int(res.success)} msg={res.message.replace(' ', '_')}")
        return res

    def _on_detach(self, _req, res):
        if not self.held:
            res.success, res.message = False, "not holding a box"
        else:
            self.stage.RemovePrim(self.joint)
            res.success, res.message = True, self.held
            self.held = ""
        self.events.append(f"GRIP_DETACH ok={int(res.success)} msg={res.message.replace(' ', '_')}")
        return res

    def _on_remove(self, msg):
        name = msg.data.strip()
        if name not in self.boxes or name in self.holders():
            self.events.append(f"BOX_REMOVE ok=0 box={name or '-'}")
            return
        x, y = REMOVED_AT[0], REMOVED_AT[1] + 0.3 * self.removed
        self.removed += 1
        prim = self.stage.GetPrimAtPath(self.boxes[name])
        # authoring the translate op on a rigid body during play teleports it in PhysX
        prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, y, L.BOX_SIZE / 2 + 0.002))
        for a in ("physics:velocity", "physics:angularVelocity"):
            attr = prim.GetAttribute(a)
            if attr:
                attr.Set(Gf.Vec3f(0, 0, 0))
        self.events.append(f"BOX_REMOVE ok=1 box={name} to={x:.2f},{y:.2f}")

    def holders(self):
        """box id -> holder name (namespace, or "robot" for the single-robot sim)."""
        return {h.held: (h.ns or "robot") for h in self.peers if h.held}

    def box_states(self, stamp=None):
        msg = BoxStates()
        msg.header.frame_id = "map"
        if stamp is not None:
            msg.header.stamp = stamp
        for name, p in self.box_positions().items():
            m = self.to_map((float(p[0]), float(p[1]), float(p[2])))
            holder = self.holders().get(name, "")
            shelf, slot = box_state.classify(m, self.slots, held=bool(holder))
            msg.boxes.append(BoxState(id=name, shelf=shelf, slot=slot, held=bool(holder), holder=holder,
                                      position=Point(x=m[0], y=m[1], z=m[2])))
        return msg

    def _make_joint(self, name):
        pl, ql = _pose(self.lift)
        pb, qb = _pose(self.boxes[name])
        qinv = ql.GetInverse()
        rel_p = Gf.Rotation(qinv).TransformDir(pb - pl)
        rel_q = qinv * qb
        j = UsdPhysics.FixedJoint.Define(self.stage, self.joint)
        j.CreateBody0Rel().SetTargets([self.lift])
        j.CreateBody1Rel().SetTargets([self.boxes[name]])
        j.CreateLocalPos0Attr(Gf.Vec3f(*rel_p))
        j.CreateLocalRot0Attr(Gf.Quatf(rel_q.GetReal(), *(float(v) for v in rel_q.GetImaginary())))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateCollisionEnabledAttr(False)

    # --- per frame -----------------------------------------------------------
    def spin(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def publish(self):
        self.pub_h.publish(Float64(data=self.lift_height()))
        self.pub_held.publish(String(data=self.held))
        if self.pub_boxes is not None:
            self.pub_boxes.publish(self.box_states(self.node.get_clock().now().to_msg()))

    def fleet_status_line(self, t):
        """`FBOXES t= robot1:lift=..,held=.. ... box_00=x,y,z@holder ...` (world frame)."""
        hs = self.holders()
        parts = [f"FBOXES t={t:.2f}"]
        parts += [f"{h.ns}:lift={h.lift_height():.3f},held={h.held or '-'}" for h in self.peers]
        for n, p in self.box_positions().items():
            parts.append(f"{n}={p[0]:.3f},{p[1]:.3f},{p[2]:.3f}@{hs.get(n, '-')}")
        return " ".join(parts)

    def status_line(self, t):
        parts = [f"BOXES t={t:.2f} lift={self.lift_height():.3f} held={self.held or '-'}"]
        for n, p in self.box_positions().items():
            parts.append(f"{n}={p[0]:.3f},{p[1]:.3f},{p[2]:.3f}")
        return " ".join(parts)

    def shutdown(self):
        self.node.destroy_node()
        if self.owns_boxes and rclpy.ok():
            rclpy.shutdown()
