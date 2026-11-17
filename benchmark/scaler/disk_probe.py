"""Print JSON with the on-disk size of the fault manager's state and the ROS logs.

Runs inside the benchmark container with stdlib only. The store directory holds
the SQLite fault store, its WAL files and the audit log; the rosbag directory
and the ROS log directory are measured separately.
"""
from __future__ import annotations
import argparse
import json
import os
import sqlite3
import sys

_TABLES = ("faults", "snapshots", "freeze_frames", "near_misses", "rosbag_files")


def dir_bytes(path):
    """Apparent size of all regular files below ``path`` (0 when it is absent)."""
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.lstat(os.path.join(root, name)).st_size
            except OSError:
                pass
    return total


def store_files(store_dir, skip):
    """Size of each regular file directly in ``store_dir`` except ``skip`` names."""
    out = {}
    try:
        names = sorted(os.listdir(store_dir))
    except OSError:
        return out
    for name in names:
        full = os.path.join(store_dir, name)
        if name not in skip and os.path.isfile(full):
            out[name] = os.lstat(full).st_size
    return out


def table_counts(db_path):
    """Row count per fault-store table, read-only; {} when the DB cannot be opened."""
    counts = {}
    try:
        con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=5)
    except sqlite3.Error:
        return counts
    try:
        for table in _TABLES:
            try:
                counts[table] = con.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0]
            except sqlite3.Error:
                pass
        try:
            counts["max_capture_id"] = con.execute(
                "SELECT COALESCE(MAX(capture_id), 0) FROM snapshots").fetchone()[0]
        except sqlite3.Error:
            pass
    finally:
        con.close()
    return counts


def db_usage(db_path):
    """Logical size of the SQLite store as a reader sees it, and bytes per table.

    A WAL file keeps its high-water size after a checkpoint, so file sizes
    overstate the data; page_count counts pages including the uncheckpointed
    ones, freelist pages are subtracted. Per-table bytes need the dbstat
    virtual table and are omitted when SQLite was built without it.
    """
    try:
        con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=5)
    except sqlite3.Error:
        return {}
    try:
        page = con.execute("PRAGMA page_size").fetchone()[0]
        pages = con.execute("PRAGMA page_count").fetchone()[0]
        free = con.execute("PRAGMA freelist_count").fetchone()[0]
        out = {"db_logical_bytes": pages * page, "db_used_bytes": (pages - free) * page}
        try:
            out["table_bytes"] = {
                name: size for name, size in con.execute(
                    "SELECT name, SUM(pgsize) FROM dbstat GROUP BY name")}
        except sqlite3.Error:
            pass
        return out
    except sqlite3.Error:
        return {}
    finally:
        con.close()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--store-dir", default="/var/lib/ros2_medkit")
    ap.add_argument("--db", default="/var/lib/ros2_medkit/faults.db")
    ap.add_argument("--rosbag-dir", default="/var/lib/ros2_medkit/rosbags")
    ap.add_argument("--log-dir", default=os.environ.get(
        "ROS_LOG_DIR", os.path.expanduser("~/.ros/log")))
    args = ap.parse_args(argv)
    skip = {os.path.basename(args.rosbag_dir)}
    files = store_files(args.store_dir, skip)
    json.dump({
        "store_files": files,
        "store_bytes": sum(files.values()),
        "tables": table_counts(args.db),
        "db": db_usage(args.db),
        "rosbag_bytes": dir_bytes(args.rosbag_dir),
        "rosbag_entries": (len(os.listdir(args.rosbag_dir))
                           if os.path.isdir(args.rosbag_dir) else 0),
        "log_bytes": dir_bytes(args.log_dir),
    }, sys.stdout)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
