#!/usr/bin/env python3
"""Trim over-captured RealSense YUV files to the best synchronized M-frame span.

Workflow:
  1. Capture N frames on every robot, where N > M.
  2. Copy each robot's raw YUV and CSV into one Results directory.
  3. Run this script to choose per-robot start offsets that minimize timestamp
     skew over a final M-frame window.

The CSV must contain frame_idx and timestamp_error_ms.  We compare
timestamp_error_ms because each robot's monotonic clock has a different epoch.
"""

import argparse
import csv
import itertools
from pathlib import Path
from statistics import mean


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", default="../Results")
    parser.add_argument("--robots", nargs="+",
                        default=["vcrobot3", "vcrobot5", "vcrobot6"])
    parser.add_argument("--suffix", default="r3",
                        help="matches files like vcrobot3_*_r3.yuv and vcrobot3_r3.csv")
    parser.add_argument("--target-frames", type=int, required=True,
                        help="final synchronized frame count M")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--format", choices=("yuv420",), default="yuv420")
    parser.add_argument("--max-search-extra", type=int, default=200,
                        help="limit offset search per robot for speed")
    parser.add_argument("--output-tag", default="sync")
    parser.add_argument("--metric", choices=("max", "p95", "mean"), default="max")
    return parser.parse_args()


def yuv420_frame_size(width, height):
    return width * height * 3 // 2


def read_csv(path):
    rows = []
    with open(path, newline="") as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            try:
                idx = int(row["frame_idx"])
                err = float(row["timestamp_error_ms"])
            except (KeyError, TypeError, ValueError):
                continue
            rows.append({
                "frame_idx": idx,
                "timestamp_error_ms": err,
                "ros_stamp_ns": row.get("ros_stamp_ns"),
                "callback_mono_ns": row.get("callback_mono_ns"),
                "clock_ns": row.get("clock_ns"),
                "target_ns": row.get("target_ns"),
            })
    rows.sort(key=lambda item: item["frame_idx"])
    return rows


def find_files(results_dir, robot, suffix):
    csv_path = results_dir / f"{robot}_{suffix}.csv"
    yuv_candidates = sorted(results_dir.glob(f"{robot}_*_{suffix}.yuv"))
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    if not yuv_candidates:
        raise FileNotFoundError(
            f"YUV not found: {results_dir / (robot + '_*_' + suffix + '.yuv')}"
        )
    return yuv_candidates[-1], csv_path


def score_window(series, offsets, target_frames, metric):
    skews = []
    for frame in range(target_frames):
        values = [
            rows[offset + frame]["timestamp_error_ms"]
            for rows, offset in zip(series, offsets)
        ]
        skews.append(max(values) - min(values))
    if metric == "mean":
        score = mean(skews)
    elif metric == "p95":
        ordered = sorted(skews)
        score = ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))]
    else:
        score = max(skews)
    return score, skews


def find_best_offsets(series, target_frames, max_search_extra, metric):
    ranges = []
    for rows in series:
        max_offset = len(rows) - target_frames
        if max_offset < 0:
            raise ValueError(
                f"not enough frames: have {len(rows)}, need {target_frames}"
            )
        ranges.append(range(min(max_offset, max_search_extra) + 1))
    best = None
    for offsets in itertools.product(*ranges):
        score, skews = score_window(series, offsets, target_frames, metric)
        if best is None or score < best["score"]:
            best = {"offsets": offsets, "score": score, "skews": skews}
    return best


def trim_yuv(src, dst, start_frame, frame_count, frame_size):
    with open(src, "rb") as in_fp, open(dst, "wb") as out_fp:
        in_fp.seek(start_frame * frame_size)
        remaining = frame_count
        chunk_frames = 8
        while remaining > 0:
            take = min(remaining, chunk_frames)
            data = in_fp.read(take * frame_size)
            if len(data) != take * frame_size:
                raise IOError(f"{src}: short read while trimming")
            out_fp.write(data)
            remaining -= take


def write_trimmed_csv(src_rows, dst, start, frame_count):
    fieldnames = [
        "frame_idx", "source_frame_idx", "timestamp_error_ms", "ros_stamp_ns",
        "callback_mono_ns", "clock_ns", "target_ns"
    ]
    with open(dst, "w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for out_idx in range(frame_count):
            row = src_rows[start + out_idx]
            writer.writerow({
                "frame_idx": out_idx,
                "source_frame_idx": row["frame_idx"],
                "timestamp_error_ms": row["timestamp_error_ms"],
                "ros_stamp_ns": row["ros_stamp_ns"],
                "callback_mono_ns": row["callback_mono_ns"],
                "clock_ns": row["clock_ns"],
                "target_ns": row["target_ns"],
            })


def write_report(path, robots, offsets, skews, series):
    with open(path, "w", newline="") as fp:
        fieldnames = (
            ["frame_idx", "skew_ms"] +
            [f"{robot}_source_frame" for robot in robots] +
            [f"{robot}_timestamp_error_ms" for robot in robots]
        )
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for frame_idx, skew in enumerate(skews):
            row = {"frame_idx": frame_idx, "skew_ms": skew}
            for robot, offset, rows in zip(robots, offsets, series):
                source = rows[offset + frame_idx]
                row[f"{robot}_source_frame"] = source["frame_idx"]
                row[f"{robot}_timestamp_error_ms"] = source["timestamp_error_ms"]
            writer.writerow(row)


def main():
    args = parse_args()
    results_dir = Path(args.results_dir).resolve()
    frame_size = yuv420_frame_size(args.width, args.height)
    inputs = []
    series = []
    for robot in args.robots:
        yuv_path, csv_path = find_files(results_dir, robot, args.suffix)
        rows = read_csv(csv_path)
        inputs.append((robot, yuv_path, csv_path))
        series.append(rows)
        print(f"{robot}: yuv={yuv_path.name}, csv={csv_path.name}, frames={len(rows)}")

    best = find_best_offsets(
        series, args.target_frames, args.max_search_extra, args.metric
    )
    offsets = best["offsets"]
    skews = best["skews"]
    passed = sum(1 for value in skews if value <= 10.0)
    ordered = sorted(skews)
    p95 = ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))]
    print("\nBest synchronized window:")
    print(f"  offsets={dict(zip(args.robots, offsets))}")
    print(
        f"  skew_ms min={min(skews):.3f}, mean={mean(skews):.3f}, "
        f"p95={p95:.3f}, max={max(skews):.3f}"
    )
    print(f"  frames <= 10ms: {passed}/{len(skews)} ({passed / len(skews) * 100:.1f}%)")

    for (robot, yuv_path, csv_path), offset, rows in zip(inputs, offsets, series):
        out_yuv = results_dir / f"{robot}_{args.width}x{args.height}_{args.output_tag}.yuv"
        out_csv = results_dir / f"{robot}_{args.output_tag}.csv"
        trim_yuv(yuv_path, out_yuv, offset, args.target_frames, frame_size)
        write_trimmed_csv(rows, out_csv, offset, args.target_frames)
        print(f"  wrote {out_yuv.name}, {out_csv.name}")

    report = results_dir / f"sync_report_{args.output_tag}.csv"
    write_report(report, args.robots, offsets, skews, series)
    print(f"  wrote {report.name}")


if __name__ == "__main__":
    main()
