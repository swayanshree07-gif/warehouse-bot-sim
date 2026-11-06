"""task_manager node: MoveBox action server on /move_box (Day 19).

  ros2 run warehouse_tasks task_manager --ros-args -p backend:=mock
  ros2 launch warehouse_tasks task_manager.launch.py backend:=ros

Parameters
  registry   path to registry.yaml (default: warehouse_bringup share/config)
  backend    "mock" (no sim, MockRobot) or "ros" (Nav2 + gripper services)
  mock_step_time  seconds per mocked step (default 0.2)
  mock_fail  "method:message" makes that mocked call fail, e.g. "navigate:unreachable"
  lidar_dock true (default): dock on the lidar rack-face measurement (Day 20);
             false: AMCL-only dock (the Day 21 behaviour)

Goals are accepted and run one at a time in arrival order (a lock queues
them). Each goal is validated against the current box belief when it starts,
so an invalid goal (unknown box/shelf, slot occupied, no free slot) aborts with
the reason in result.message. After a success the belief is updated.
Logs one `TASK_RESULT {json}` line per goal.
"""
import json
import os
import signal
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from warehouse_interfaces.action import MoveBox

from .mission import run_move_box
from .mock_robot import MockRobot
from .registry import Registry, TaskError


def default_registry():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("warehouse_bringup"), "config", "registry.yaml")
    except Exception:
        return ""


class TaskManager(Node):
    def __init__(self):
        super().__init__("task_manager")
        self.declare_parameter("registry", default_registry())
        self.declare_parameter("backend", "mock")
        self.declare_parameter("mock_step_time", 0.2)
        self.declare_parameter("mock_fail", "")
        self.declare_parameter("lidar_dock", True)
        path = self.get_parameter("registry").value
        self.reg = Registry.load(path)
        backend = self.get_parameter("backend").value
        if backend == "mock":
            fail = self.get_parameter("mock_fail").value
            fail_at = dict([fail.split(":", 1)]) if fail else {}
            self.robot = MockRobot(self.reg, step_time=self.get_parameter("mock_step_time").value,
                                   fail_at=fail_at)
        elif backend == "ros":
            from .ros_robot import RosRobot
            self.robot = RosRobot(self, registry=self.reg,
                                  lidar_dock=self.get_parameter("lidar_dock").value)
        else:
            raise ValueError(f"unknown backend '{backend}' (mock|ros)")
        self.lock = threading.Lock()
        self.server = ActionServer(
            self, MoveBox, "move_box", execute_callback=self.execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda h: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f"task_manager ready: backend={backend} registry={path} "
                               f"boxes={len(self.reg.boxes)}")

    def execute(self, gh):
        g = gh.request
        with self.lock:   # one task at a time; later goals wait here (queue)
            res = MoveBox.Result()
            self.get_logger().info(f"goal: {g.box_id} -> {g.target_shelf} slot {g.target_slot}")
            try:
                plan = self.reg.plan(g.box_id, g.target_shelf, g.target_slot)
            except TaskError as e:
                res.success, res.message = False, str(e)
                return self._finish(gh, res, g, aborted=True)

            def feedback(state, progress):
                fb = MoveBox.Feedback()
                fb.state, fb.progress = state, float(progress)
                gh.publish_feedback(fb)
                self.get_logger().info(f"state {state} ({progress:.2f})")

            out = run_move_box(self.robot, plan, self.reg.lift_clearance, feedback=feedback,
                               is_canceled=lambda: gh.is_cancel_requested)
            res.success, res.message = out.success, out.message
            res.final_shelf, res.final_slot = out.shelf, out.slot
            res.place_error = float(out.place_error)
            if out.success:
                self.reg.move(g.box_id, out.shelf, out.slot)
            return self._finish(gh, res, g, aborted=not out.success, canceled=out.canceled)

    def _finish(self, gh, res, g, aborted=False, canceled=False):
        if canceled:
            gh.canceled()
        elif aborted:
            gh.abort()
        else:
            gh.succeed()
        info = {"box_id": g.box_id, "target_shelf": g.target_shelf, "target_slot": g.target_slot,
                "success": res.success, "message": res.message, "final_shelf": res.final_shelf,
                "final_slot": res.final_slot, "place_error": round(res.place_error, 4),
                "canceled": canceled}
        self.get_logger().info("TASK_RESULT " + json.dumps(info))
        return res


def main():
    # Own SIGINT/SIGTERM handling: rclpy's handler raises KeyboardInterrupt in
    # whatever the main thread is doing, which made shutdown print tracebacks
    # from executor/future destructors. Here a signal only sets a flag.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = TaskManager()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        while rclpy.ok() and not stop.is_set():
            ex.spin_once(timeout_sec=0.1)
    except ExternalShutdownException:
        pass
    finally:
        node.get_logger().info("task_manager shutting down")
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
