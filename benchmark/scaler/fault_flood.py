"""Report faults to the fault manager and wait for each capture to land.

Runs inside the benchmark container. Prints one JSON line with the counts.

Modes:
  distinct  ``--count`` faults with distinct codes, one report each.
  repeat    ``--count`` reports of one code (``--code``).

After each report the script waits until the store shows its capture, so the
capture queue (depth 16, reject_newest by default) never drops one. With
``--wait-for rosbag`` it waits for a new recording row instead.
"""
from __future__ import annotations
import argparse
import json
import sqlite3
import sys
import time

import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.srv import ClearFault, ReportFault

_CRITICAL = 3
_EVENT_FAILED = 0
_SERVICE = "/fault_manager/report_fault"
_CLEAR_SERVICE = "/fault_manager/clear_fault"


def _scalar(db, sql):
    try:
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True, timeout=5)
        try:
            return con.execute(sql).fetchone()[0]
        finally:
            con.close()
    except sqlite3.Error:
        return 0


# What each wait-for value counts in the store; the report waits until it is
# at least the number of reports sent so far.
_PROGRESS_SQL = {
    "freeze_frame": "SELECT COUNT(*) FROM freeze_frames",
    "capture": "SELECT COALESCE(MAX(capture_id), 0) FROM snapshots",
    "occurrence": "SELECT COALESCE(SUM(occurrence_count), 0) FROM faults",
    "rosbag": "SELECT COUNT(*) FROM rosbag_files",
    "none": None,
}


def wait_until(predicate, timeout_s, poll_s=0.02):
    """Poll ``predicate`` until true or the deadline; return whether it held."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(poll_s)
    return predicate()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["distinct", "repeat"], required=True)
    ap.add_argument("--count", type=int, required=True)
    ap.add_argument("--code", default="BENCH_REPEAT")
    ap.add_argument("--prefix", default="BENCH_FAULT_")
    ap.add_argument("--source", default="/bench_source")
    ap.add_argument("--db", default="/var/lib/ros2_medkit/faults.db")
    ap.add_argument("--wait-for", choices=sorted(_PROGRESS_SQL), default="freeze_frame")
    ap.add_argument("--clear-between", action="store_true",
                    help="Clear the fault after each report, so the next report is a new occurrence.")
    ap.add_argument("--wait-timeout", type=float, default=30.0)
    ap.add_argument("--start-timeout", type=float, default=120.0)
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Node("bench_fault_flood")
    client = node.create_client(ReportFault, _SERVICE)
    if not client.wait_for_service(timeout_sec=args.start_timeout):
        print(json.dumps({"error": f"{_SERVICE} not available"}))
        return 1
    # The capture reads the source's topics from the graph, so the publisher
    # must be discovered before the first report.
    if not wait_until(lambda: any(
            i.node_name == "bench_source"
            for i in node.get_publishers_info_by_topic("/bench_source/state")),
            args.start_timeout):
        print(json.dumps({"error": "bench_source topic not discovered"}))
        return 1

    clear_client = node.create_client(ClearFault, _CLEAR_SERVICE)
    if args.clear_between and not clear_client.wait_for_service(timeout_sec=args.start_timeout):
        print(json.dumps({"error": f"{_CLEAR_SERVICE} not available"}))
        return 1
    sql = _PROGRESS_SQL[args.wait_for]
    # The store may already hold rows (a baseline); progress is measured from it.
    base = _scalar(args.db, sql) if sql else 0
    accepted = landed = 0
    for i in range(1, args.count + 1):
        code = args.code if args.mode == "repeat" else f"{args.prefix}{i}"
        req = ReportFault.Request()
        req.fault_code = code
        req.event_type = _EVENT_FAILED
        req.severity = _CRITICAL
        req.description = f"benchmark fault {i}"
        req.source_id = args.source
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        if fut.result() is not None and fut.result().accepted:
            accepted += 1
        if sql and wait_until(lambda: _scalar(args.db, sql) >= base + i,
                              args.wait_timeout):
            landed += 1
        if args.clear_between:
            clear_req = ClearFault.Request()
            clear_req.fault_code = code
            cfut = clear_client.call_async(clear_req)
            rclpy.spin_until_future_complete(node, cfut, timeout_sec=30.0)
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps({"reported": args.count, "accepted": accepted,
                      "landed": landed, "waited_for": args.wait_for}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
