"""Validate a recorded rosbag (Day 25).

  python3 scripts/check_bag.py BAG_DIR [--max-mb 300] [--need /clock,/odom,...]

Reads BAG_DIR/metadata.yaml (written when the recorder closes the bag) and
checks: total size of the bag files < --max-mb, every --need topic was
recorded with >= 1 message. Prints `BAG {json}` (duration, size, per-topic
counts) and `BAG_PASS` / `BAG_FAIL <reasons>`. Pure Python + PyYAML.
"""
import argparse
import json
import os
import sys

import yaml

DEFAULT_NEED = ("/clock,/tf,/tf_static,/odom,/scan,/joint_states,/cmd_vel,/map,"
                "/boxes,/lift/height,/move_box/_action/status,/move_box/_action/feedback")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--max-mb", type=float, default=300.0)
    ap.add_argument("--need", default=DEFAULT_NEED)
    a = ap.parse_args()
    meta_path = os.path.join(a.bag, "metadata.yaml")
    if not os.path.isfile(meta_path):
        print(f"BAG_FAIL no metadata.yaml in {a.bag} (recorder not closed cleanly?)")
        return 1
    info = yaml.safe_load(open(meta_path))["rosbag2_bagfile_information"]
    counts = {t["topic_metadata"]["name"]: t["message_count"] for t in info["topics_with_message_count"]}
    size = sum(os.path.getsize(os.path.join(a.bag, f)) for f in os.listdir(a.bag))
    dur = info["duration"]["nanoseconds"] / 1e9
    print("BAG " + json.dumps({"dir": a.bag, "storage": info["storage_identifier"],
                               "duration_s": round(dur, 1), "size_mb": round(size / 1e6, 2),
                               "messages": info["message_count"], "topics": counts}), flush=True)
    bad = []
    if size / 1e6 >= a.max_mb:
        bad.append(f"size {size / 1e6:.1f} MB >= {a.max_mb}")
    bad += [f"{t} missing/empty" for t in filter(None, a.need.split(",")) if counts.get(t, 0) < 1]
    if bad:
        print("BAG_FAIL " + "; ".join(bad))
        return 1
    print(f"BAG_PASS {len(counts)} topics, {info['message_count']} msgs, {size / 1e6:.1f} MB, {dur:.0f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
