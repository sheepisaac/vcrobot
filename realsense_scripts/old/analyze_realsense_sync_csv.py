#!/usr/bin/env python3
"""Analyze timestamp-based RealSense synchronization CSV files.

Use this on CSV files generated next to synchronized YUV recordings.

Important: slave monotonic clock_ns values are local to each robot, and RealSense
HARDWARE_CLOCK ros_stamp_ns values are also not guaranteed to share a global epoch.
For cross-robot sync quality, compare timestamp_error_ms, which is each selected
frame's local error from the master-scheduled local target time.
"""

import argparse
import csv
from pathlib import Path
from statistics import mean


def robot_name(path):
    stem = Path(path).name
    for marker in ("_r3", "_r5", "_r6"):
        if marker in stem:
            return marker.strip("_")
    parts = stem.split("_")
    for part in parts:
        if part.startswith("r") and part[1:].isdigit():
            return part
    return Path(path).stem


def read_csv(path):
    rows = []
    with open(path, newline="") as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            try:
                frame_idx = int(row["frame_idx"])
            except (KeyError, TypeError, ValueError):
                continue
            error = parse_float(row.get("timestamp_error_ms"))
            callback = parse_int(row.get("callback_mono_ns"))
            ros_stamp = parse_int(row.get("ros_stamp_ns"))
            rows.append({
                "frame_idx": frame_idx,
                "timestamp_error_ms": error,
                "callback_mono_ns": callback,
                "ros_stamp_ns": ros_stamp,
            })
    return rows


def parse_float(value):
    if value in (None, ""):
        return None
    try:
        return float(value)
    except ValueError:
        return None


def parse_int(value):
    if value in (None, ""):
        return None
    try:
        return int(float(value))
    except ValueError:
        return None


def summarize(paths):
    series = {}
    for path in paths:
        name = robot_name(path)
        rows = read_csv(path)
        if not rows:
            print(f"{path}: no readable rows")
            continue
        series[name] = rows
        errors = [row["timestamp_error_ms"] for row in rows
                  if row["timestamp_error_ms"] is not None]
        print(f"{name}: {path}")
        print(f"  frames={len(rows)}")
        if errors:
            print(
                "  timestamp_error_ms: "
                f"min={min(errors):+.3f}, max={max(errors):+.3f}, "
                f"mean={mean(errors):+.3f}"
            )
        else:
            print("  timestamp_error_ms: missing")
    if len(series) < 2:
        return

    common = sorted(set.intersection(*[
        {row["frame_idx"] for row in rows}
        for rows in series.values()
    ]))
    skews = []
    worst = []
    by_robot = {
        name: {row["frame_idx"]: row for row in rows}
        for name, rows in series.items()
    }
    for frame_idx in common:
        values = []
        detail = {}
        for name, rows in by_robot.items():
            value = rows[frame_idx]["timestamp_error_ms"]
            if value is None:
                break
            values.append(value)
            detail[name] = value
        if len(values) != len(series):
            continue
        skew = max(values) - min(values)
        skews.append(skew)
        worst.append((skew, frame_idx, detail))
    if not skews:
        print("\nNo common frames with timestamp_error_ms across all CSVs.")
        return
    skews_sorted = sorted(skews)
    p95 = skews_sorted[min(len(skews_sorted) - 1, int(len(skews_sorted) * 0.95))]
    pass_10ms = sum(1 for value in skews if value <= 10.0)
    print("\nCross-robot sync from timestamp_error_ms:")
    print(f"  common_frames={len(skews)}")
    print(
        f"  skew_ms: min={min(skews):.3f}, mean={mean(skews):.3f}, "
        f"p95={p95:.3f}, max={max(skews):.3f}"
    )
    print(f"  frames <= 10ms: {pass_10ms}/{len(skews)} "
          f"({pass_10ms / len(skews) * 100:.1f}%)")
    print("  worst frames:")
    for skew, frame_idx, detail in sorted(worst, reverse=True)[:10]:
        details = ", ".join(
            f"{name}={value:+.3f}ms" for name, value in sorted(detail.items())
        )
        print(f"    frame {frame_idx}: skew={skew:.3f}ms; {details}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", help="CSV files from robot recordings")
    args = parser.parse_args()
    summarize(args.csv)


if __name__ == "__main__":
    main()
