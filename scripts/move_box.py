"""MoveBox action client (Day 19).

  python3 scripts/move_box.py BOX SHELF [SLOT] [--cancel-after S] [--timeout S]
Sends a goal to /move_box, prints `MOVE_BOX_FEEDBACK state= progress=` lines
and one `MOVE_BOX_RESULT {json}` line (status = succeeded|aborted|canceled).
Exit 0 only if the goal succeeded. Needs `source env.sh` and a built ros_ws.
"""
import argparse
import json
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from warehouse_interfaces.action import MoveBox

STATUS = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
          GoalStatus.STATUS_CANCELED: "canceled"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("box")
    ap.add_argument("shelf")
    ap.add_argument("slot", nargs="?", type=int, default=-1)
    ap.add_argument("--cancel-after", type=float, default=None)
    ap.add_argument("--timeout", type=float, default=600.0)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("move_box_client")
    cli = ActionClient(node, MoveBox, "move_box")
    try:
        if not cli.wait_for_server(timeout_sec=20.0):
            print("MOVE_BOX_RESULT " + json.dumps({"status": "no_server"}), flush=True)
            return 1

        def on_fb(m):
            print(f"MOVE_BOX_FEEDBACK state={m.feedback.state} progress={m.feedback.progress:.2f}", flush=True)
        fut = cli.send_goal_async(MoveBox.Goal(box_id=a.box, target_shelf=a.shelf, target_slot=a.slot),
                                  feedback_callback=on_fb)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
        h = fut.result()
        if h is None or not h.accepted:
            print("MOVE_BOX_RESULT " + json.dumps({"status": "rejected"}), flush=True)
            return 1
        rfut = h.get_result_async()
        t0 = time.time()
        canceled = False
        while not rfut.done() and time.time() - t0 < a.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if a.cancel_after is not None and not canceled and time.time() - t0 > a.cancel_after:
                h.cancel_goal_async()
                canceled = True
        if not rfut.done():
            h.cancel_goal_async()
            print("MOVE_BOX_RESULT " + json.dumps({"status": "client_timeout"}), flush=True)
            return 1
        r = rfut.result()
        res = r.result
        out = {"status": STATUS.get(r.status, str(r.status)), "success": res.success,
               "message": res.message, "final_shelf": res.final_shelf, "final_slot": res.final_slot,
               "place_error": round(res.place_error, 4)}
        print("MOVE_BOX_RESULT " + json.dumps(out), flush=True)
        return 0 if r.status == GoalStatus.STATUS_SUCCEEDED and res.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
