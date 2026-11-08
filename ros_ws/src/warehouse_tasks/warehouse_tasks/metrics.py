"""Fleet throughput metrics (Day 18), pure Python.

compute_metrics(tasks, robot_time, sim_time_s) where
- tasks: iterable with .state, .t_submit, .t_assign, .t_done, .robot
- robot_time: {robot: {"idle": s, "busy": s, "traffic_wait": s}}
Definitions:
- tasks_per_hour = done / sim_time * 3600
- task time = t_done - t_assign (assignment to drop finished)
- queue wait = t_assign - t_submit (time a task waited for a free robot)
- idle % = time with no task / sim_time
- traffic wait = time a robot had a task and a goal but did not move
  (blocked by a reservation or yielding in a deadlock).
"""

METRIC_KEYS = ("tasks_per_hour", "mean_task_time_s", "max_task_time_s",
               "mean_queue_wait_s", "mean_idle_pct", "per_robot")


def _mean(xs):
    return round(sum(xs) / len(xs), 2) if xs else 0.0


def compute_metrics(tasks, robot_time, sim_time_s):
    done = [t for t in tasks if t.state == "done"]
    ttime = [t.t_done - t.t_assign for t in done]
    qwait = [t.t_assign - t.t_submit for t in done]
    per = {}
    for name, rt in sorted(robot_time.items()):
        n = sum(t.robot == name for t in done)
        per[name] = {
            "tasks_done": n,
            "idle_s": round(rt["idle"], 2),
            "busy_s": round(rt["busy"], 2),
            "traffic_wait_s": round(rt["traffic_wait"], 2),
            "idle_pct": round(100.0 * rt["idle"] / sim_time_s, 1) if sim_time_s else 0.0,
        }
    return {
        "tasks_per_hour": round(len(done) * 3600.0 / sim_time_s, 1) if sim_time_s else 0.0,
        "mean_task_time_s": _mean(ttime),
        "max_task_time_s": round(max(ttime), 2) if ttime else 0.0,
        "mean_queue_wait_s": _mean(qwait),
        "mean_idle_pct": _mean([p["idle_pct"] for p in per.values()]),
        "total_traffic_wait_s": round(sum(p["traffic_wait_s"] for p in per.values()), 2),
        "per_robot": per,
    }
