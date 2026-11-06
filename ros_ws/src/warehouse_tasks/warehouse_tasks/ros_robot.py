"""Real robot backend for the MoveBox state machine (Day 19, used from Day 21).

All calls block the calling thread (the action server's execute callback) and
wait on futures/messages that the node's MultiThreadedExecutor delivers in
other threads, so every subscription/client here uses a reentrant group.

  navigate  nav2_msgs/NavigateToPose on `navigate_to_pose` (map frame)
  dock      at the approach pose, face the rack and measure its front face with
            the lidar (rack_align, Day 20): that gives the dock pose relative to
            the robot, which becomes an odom-frame target; back up first if the
            lateral error is large, then line-track forward to it and turn to
            its yaw. Falls back to the AMCL dock (TF map->base_link) if the rack
            is not found or the pose is not a registry dock pose.
  undock    reverse straight to the approach pose, keeping the heading
  lift      std_msgs/Float64 `lift/command`, wait for `lift/height`
  attach    std_srvs/Trigger `gripper/attach` / `gripper/detach`
"""
import math
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from . import rack_align
from .line_track import track_cmd, wrap


def _wait(fut, timeout):
    t0 = time.time()
    while not fut.done():
        if time.time() - t0 > timeout:
            return False
        time.sleep(0.01)
    return True


def _standoffs(slot):
    """(approach, dock) distance from the slot centre, from the registry poses."""
    c = slot["center"]
    return (math.hypot(slot["approach"][0] - c[0], slot["approach"][1] - c[1]),
            math.hypot(slot["dock"][0] - c[0], slot["dock"][1] - c[1]))


class RosRobot:
    def __init__(self, node, nav_timeout=120.0, dock_tol=0.02, yaw_tol_deg=1.5, registry=None,
                 lidar_dock=True, backup_lat=0.02):
        self.node = node
        self.lidar_dock = lidar_dock and registry is not None
        self.backup_lat = backup_lat
        # registry dock pose -> (shelf dict, slot dict), to find the rack geometry
        self.docks = {}
        if registry is not None:
            for sh in registry.shelves.values():
                for sl in sh["slots"]:
                    self.docks[tuple(round(float(v), 3) for v in sl["dock"])] = (sh, sl)
        self.scan, self.scan_count = None, 0
        self.last_dock = {}
        self.nav_timeout = nav_timeout
        self.dock_tol = dock_tol
        self.yaw_tol = math.radians(yaw_tol_deg)
        cg = ReentrantCallbackGroup()
        self.nav = ActionClient(node, NavigateToPose, "navigate_to_pose", callback_group=cg)
        self.cmd_pub = node.create_publisher(Twist, "cmd_vel", 10)
        self.lift_pub = node.create_publisher(Float64, "lift/command", 10)
        self.height = None
        node.create_subscription(Float64, "lift/height", self._on_height, 10, callback_group=cg)
        node.create_subscription(LaserScan, "scan", self._on_scan, 5, callback_group=cg)
        self.cli = {v: node.create_client(Trigger, f"gripper/{v}", callback_group=cg)
                    for v in ("attach", "detach")}
        self.tf_buf = tf2_ros.Buffer()
        # the node's MultiThreadedExecutor delivers /tf (a second executor thread on
        # the same node would conflict)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, node)

    def _on_height(self, msg):
        self.height = msg.data

    def _on_scan(self, msg):
        self.scan = msg
        self.scan_count += 1

    # --- helpers -------------------------------------------------------------
    def pose(self, frame="map", child="base_link"):
        """(x, y, yaw) of `child` in `frame` (default base_link in map), or None."""
        try:
            t = self.tf_buf.lookup_transform(frame, child, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        p, q = t.transform.translation, t.transform.rotation
        return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    def stop(self):
        for _ in range(3):
            self.cmd(0, 0)
            time.sleep(0.02)

    def _turn_to(self, yaw, timeout=30.0, frame="map"):
        t0 = time.time()
        while time.time() - t0 < timeout:
            p = self.pose(frame)
            if p is None:
                time.sleep(0.05)
                continue
            e = wrap(yaw - p[2])
            if abs(e) < self.yaw_tol:
                self.stop()
                return True, ""
            w = max(-0.6, min(0.6, 1.5 * e)) if abs(e) > 0.15 else math.copysign(0.12, e)
            self.cmd(0, w)
            time.sleep(0.03)
        self.stop()
        return False, "turn timeout"

    def _track(self, x, y, line_yaw, reverse, v_max, timeout=40.0, frame="map"):
        """Follow the line through (x, y) with direction line_yaw until the
        along-track distance is used up (line_track.track_cmd)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            p = self.pose(frame)
            if p is None:
                time.sleep(0.05)
                continue
            v, w, done, rem, lat = track_cmd(p, (x, y), line_yaw, reverse=reverse, v_max=v_max)
            if done:
                self.stop()
                return True, f"remaining={rem:+.3f} lateral={lat:+.3f}"
            self.cmd(v, w)
            time.sleep(0.03)
        self.stop()
        return False, "line timeout"

    # --- backend interface -----------------------------------------------------
    def navigate(self, pose):
        if not self.nav.wait_for_server(timeout_sec=10.0):
            return False, "navigate_to_pose server not available"
        g = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.position.x, ps.pose.position.y = float(pose[0]), float(pose[1])
        yaw = math.radians(pose[2])
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        g.pose = ps
        fut = self.nav.send_goal_async(g)
        if not _wait(fut, 10.0) or not fut.result().accepted:
            return False, "navigation goal rejected"
        handle = fut.result()
        rfut = handle.get_result_async()
        if not _wait(rfut, self.nav_timeout):
            handle.cancel_goal_async()
            return False, "navigation timeout"
        status = rfut.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            return False, f"navigation failed (status {status})"
        return True, ""

    def _measure_rack(self, shelf, slot, dock_yaw_deg):
        """Lidar dock target in the odom frame: ((x, y, yaw), errors dict) or
        (None, reason). The robot must be stopped, roughly facing the rack."""
        n0 = self.scan_count
        t0 = time.time()
        while self.scan_count < n0 + 2:          # a scan taken after stopping
            if time.time() - t0 > 3.0:
                return None, "no /scan"
            time.sleep(0.02)
        scan = self.scan
        odom = self.pose("odom")
        if odom is None:
            return None, "no TF odom->base_link"
        laser = self.pose("base_link", "laser") or (0.2, 0.0, 0.0)
        lat, depth, flen = rack_align.slot_geometry(shelf, slot, dock_yaw_deg)
        approach_off, dock_off = _standoffs(slot)
        meas, why = rack_align.measure_face(
            list(scan.ranges), scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
            laser=laser, expect_dist=approach_off - depth / 2, face_len=flen)
        if meas is None:
            return None, why
        rel, err = rack_align.dock_in_base(meas, lat, depth, dock_off)
        err["spacing"] = (meas["spacing_lo"], meas["spacing_hi"])
        return (rack_align.compose(odom, rel), err), ""

    def _dock_lidar(self, pose, shelf, slot):
        yaw = math.radians(pose[2])
        ok, msg = self._turn_to(yaw)
        if not ok:
            return ok, msg, True
        res, why = self._measure_rack(shelf, slot, pose[2])
        if res is None:
            self.node.get_logger().warn(f"lidar dock: {why}; using the AMCL dock")
            return False, why, False
        (tx, ty, tyaw), err = res
        self.last_dock = {"target_odom": (tx, ty, tyaw), **err}
        self.node.get_logger().info(
            f"lidar dock: lateral {err['lateral']:+.3f} m, yaw {math.degrees(err['yaw']):+.2f} deg, "
            f"to go {err['along']:.3f} m, corner spacing {err['spacing'][0]:.3f}/{err['spacing'][1]:.3f}")
        ux, uy = math.cos(tyaw), math.sin(tyaw)
        backed = ""
        if abs(err["lateral"]) > self.backup_lat:
            # too little run-up to remove the offset: reverse along the corrected
            # line (that converges too), then dock forward from further out
            approach_off, dock_off = _standoffs(slot)
            back = approach_off - dock_off + 0.35
            ok, m = self._track(tx - back * ux, ty - back * uy, tyaw, reverse=True, v_max=0.12,
                                frame="odom")
            if not ok:
                return ok, "lidar back-up: " + m, True
            backed = " backed-up"
        ok, m = self._track(tx, ty, tyaw, reverse=False, v_max=0.12, frame="odom")
        if not ok:
            return ok, m, True
        ok, m2 = self._turn_to(tyaw, frame="odom")
        lat0 = err["lateral"]
        return ok, (f"lidar{backed} lat0={lat0:+.3f} " + m) if ok else m2, True

    def dock(self, pose):
        """Approach pose -> dock pose. Lidar rack-face dock if this is a
        registry dock pose, else (or if the rack is not seen) the AMCL dock:
        face the dock yaw, track the access line forward to the dock point,
        then correct the final yaw."""
        if self.pose() is None:
            return False, "no TF map->base_link"
        x, y, yaw = pose[0], pose[1], math.radians(pose[2])
        key = tuple(round(float(v), 3) for v in pose)
        self.last_dock = {}
        if self.lidar_dock and key in self.docks:
            ok, msg, final = self._dock_lidar(pose, *self.docks[key])
            if final:
                return ok, msg
        ok, msg = self._turn_to(yaw)
        if not ok:
            return ok, msg
        ok, msg = self._track(x, y, yaw, reverse=False, v_max=0.12)
        if not ok:
            return ok, msg
        ok, msg2 = self._turn_to(yaw)
        return ok, msg if ok else msg2

    def undock(self, pose):
        """Back out along the access line to the approach pose (robot keeps facing the rack)."""
        if self.pose() is None:
            return False, "no TF map->base_link"
        return self._track(pose[0], pose[1], math.radians(pose[2]), reverse=True, v_max=0.15)

    def lift(self, height, tol=0.01, timeout=15.0):
        t0, last, inside = time.time(), 0.0, None
        while time.time() - t0 < timeout:
            if time.time() - last > 1.0:
                self.lift_pub.publish(Float64(data=float(height)))
                last = time.time()
            h = self.height
            if h is not None and abs(h - height) <= tol:
                inside = inside or time.time()
                if time.time() - inside > 0.5:
                    return True, f"h={h:.3f}"
            else:
                inside = None
            time.sleep(0.03)
        return False, f"lift timeout (height {self.height})"

    def _trigger(self, verb):
        cli = self.cli[verb]
        if not cli.wait_for_service(timeout_sec=5.0):
            return False, f"gripper/{verb} not available"
        fut = cli.call_async(Trigger.Request())
        if not _wait(fut, 10.0):
            return False, f"gripper/{verb} timeout"
        r = fut.result()
        return bool(r.success), r.message

    def attach(self):
        return self._trigger("attach")

    def detach(self):
        return self._trigger("detach")

    def box_position(self, _box_id):
        return None   # ground truth comes with the /boxes topic (Day 23)
