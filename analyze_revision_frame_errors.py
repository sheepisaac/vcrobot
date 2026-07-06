#!/usr/bin/env python3
"""Plot frame-wise synchronization error curves for revision result folders.

The script scans revision folders such as r02 ... r11_20260702, reads each
camera CSV, and plots per-frame timestamp error in milliseconds.

Default metric:
  abs(pts_time(camera, frame) - median_pts_time(all cameras, frame)) * 1000

This gives one line per camera/robot in each revision.  The plot is intended
for report figures showing frame-index-wise synchronization behavior.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path
from statistics import median, stdev
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


DEFAULT_RESULTS_DIR = Path(__file__).resolve().parent.parent / "Results"
DEFAULT_REVISIONS = [
    "r02", "r03", "r04", "r05", "r06", "r07", "r08", "r09", "r10", "r11_20260702"
]
DEFAULT_EXCLUDE = {
    "r11_20260702": {"vcrobot2"},
    "r11": {"vcrobot2"},
}


def natural_key(text: str):
    return [int(part) if part.isdigit() else part.lower()
            for part in re.split(r"(\d+)", text)]


def revision_label(folder: Path) -> str:
    match = re.match(r"r0*([0-9]+)", folder.name.lower())
    if match:
        return f"r{int(match.group(1)):02d}"
    return folder.name


def series_label(path: Path) -> str:
    match = re.search(r"(vcrobot\d+)", path.name, re.IGNORECASE)
    if match:
        return match.group(1).lower()
    stem = path.name
    for suffix in (".yuv.csv", ".yuyv.csv", ".rgb.csv", ".csv"):
        if stem.endswith(suffix):
            stem = stem[:-len(suffix)]
            break
    return stem


def read_pts_time(csv_path: Path) -> List[float]:
    """Read pts_time seconds from the CSV variants produced by our scripts."""
    values: List[float] = []
    with csv_path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            return values
        fieldnames = {name.lower(): name for name in reader.fieldnames}
        pts_field = fieldnames.get("pts_time")
        if pts_field is None:
            # Older ROS2 logs may not have pts_time.  Prefer frame_late/gate
            # style fields only if present and numeric.
            pts_field = (
                fieldnames.get("timestamp_error_ms")
                or fieldnames.get("frame_late_from_gate")
                or fieldnames.get("ros_stamp_ns")
            )
        for row in reader:
            raw = row.get(pts_field, "") if pts_field else ""
            if raw is None or str(raw).strip() == "":
                continue
            try:
                value = float(raw)
            except ValueError:
                continue
            if not math.isfinite(value):
                continue
            # ns field fallback: convert absolute ns to seconds and later
            # normalize by frame index like normal pts_time.
            if pts_field and pts_field.lower().endswith("_ns"):
                value /= 1e9
            # ms fallback fields are already errors, but keeping them in
            # seconds would be wrong.  Convert them to seconds so downstream
            # ms output remains consistent.
            if pts_field in ("timestamp_error_ms", "frame_late_from_gate"):
                value /= 1000.0
            values.append(value)
    return values


def load_revision(folder: Path, exclude_labels: Iterable[str]) -> Dict[str, List[float]]:
    exclude = {item.lower() for item in exclude_labels}
    series: Dict[str, List[float]] = {}
    for csv_path in sorted(folder.glob("*.csv"), key=lambda path: natural_key(path.name)):
        label = series_label(csv_path)
        if label.lower() in exclude:
            continue
        pts = read_pts_time(csv_path)
        if not pts:
            continue
        if label in series:
            # Avoid accidental overwrite for anonymous r07-style filenames.
            label = f"{label}_{len(series) + 1}"
        series[label] = pts
    return series


def frame_errors_ms(
    series: Dict[str, List[float]],
    metric: str,
    max_frames: Optional[int] = None,
) -> Tuple[List[int], Dict[str, List[float]], List[float]]:
    """Return frame indices, per-series error curves, and per-frame skew."""
    if not series:
        return [], {}, []
    common = min(len(values) for values in series.values())
    if max_frames is not None:
        common = min(common, max_frames)
    frame_indices = list(range(common))
    errors = {label: [] for label in series}
    skews: List[float] = []
    for frame_idx in frame_indices:
        values = [series[label][frame_idx] for label in series]
        center = median(values)
        min_v, max_v = min(values), max(values)
        skews.append((max_v - min_v) * 1000.0)
        for label, pts in series.items():
            if metric == "median_abs":
                error = abs(pts[frame_idx] - center) * 1000.0
            elif metric == "first_abs":
                reference = values[0]
                error = abs(pts[frame_idx] - reference) * 1000.0
            elif metric == "relative_zero":
                # Remove each camera's first-frame offset, then compare to
                # median at each frame. Useful for measuring drift only.
                normalized_values = [
                    series[item][frame_idx] - series[item][0]
                    for item in series
                ]
                normalized_center = median(normalized_values)
                error = abs((pts[frame_idx] - pts[0]) - normalized_center) * 1000.0
            else:
                raise ValueError(f"unknown metric: {metric}")
            errors[label].append(error)
    return frame_indices, errors, skews


def frame_stddev_ms(
    series: Dict[str, List[float]],
    max_frames: Optional[int] = None,
    relative_zero: bool = False,
) -> Tuple[List[int], List[float]]:
    """Return per-frame sample standard deviation across cameras in ms."""
    if len(series) < 2:
        return [], []
    common = min(len(values) for values in series.values())
    if max_frames is not None:
        common = min(common, max_frames)
    frame_indices = list(range(common))
    stddevs: List[float] = []
    labels = list(series)
    for frame_idx in frame_indices:
        if relative_zero:
            values = [
                series[label][frame_idx] - series[label][0]
                for label in labels
            ]
        else:
            values = [series[label][frame_idx] for label in labels]
        stddevs.append(stdev(values) * 1000.0)
    return frame_indices, stddevs


def frame_robot_stddev_component_ms(
    series: Dict[str, List[float]],
    max_frames: Optional[int] = None,
    relative_zero: bool = False,
) -> Tuple[List[int], Dict[str, List[float]], List[float]]:
    """Return each robot's per-frame sample-standard-deviation component.

    For one frame with N robot timestamps x_i, the sample standard deviation is:

        sqrt(sum((x_i - mean)^2) / (N - 1))

    To draw one line per robot, this returns:

        abs(x_i - mean) / sqrt(N - 1)

    in milliseconds.  The quadrature sum of these per-robot components equals
    the frame-wise sample standard deviation.
    """
    if len(series) < 2:
        return [], {}, []
    common = min(len(values) for values in series.values())
    if max_frames is not None:
        common = min(common, max_frames)
    labels = list(series)
    frame_indices = list(range(common))
    deviations = {label: [] for label in labels}
    stddevs: List[float] = []
    for frame_idx in frame_indices:
        if relative_zero:
            values = [
                series[label][frame_idx] - series[label][0]
                for label in labels
            ]
        else:
            values = [series[label][frame_idx] for label in labels]
        mean_value = sum(values) / len(values)
        stddevs.append(stdev(values) * 1000.0)
        denominator = math.sqrt(len(values) - 1)
        for label, value in zip(labels, values):
            deviations[label].append(abs(value - mean_value) / denominator * 1000.0)
    return frame_indices, deviations, stddevs


def summarize(label: str, skews: Sequence[float]) -> str:
    if not skews:
        return f"{label}: no data"
    sorted_skews = sorted(skews)
    p95 = sorted_skews[min(len(sorted_skews) - 1, math.ceil(len(sorted_skews) * 0.95) - 1)]
    avg = sum(skews) / len(skews)
    return (
        f"{label}: frames={len(skews)} "
        f"avg_skew={avg:.3f}ms median={median(skews):.3f}ms "
        f"p95={p95:.3f}ms max={max(skews):.3f}ms"
    )


def rounded_axis_max(value: float, base: int = 10, minimum: Optional[int] = None) -> int:
    """Round an axis maximum up to a clean base unit.

    The user-facing intent is "round at the ones place" for report-ready axes;
    using ceiling to the next 10 keeps the axis from clipping the data.
    """
    if value <= 0 or not math.isfinite(value):
        result = base
    else:
        result = int(math.ceil(value / base) * base)
    if minimum is not None:
        result = max(result, minimum)
    return result


def max_error_value(errors: Dict[str, List[float]]) -> float:
    values = [value for curve in errors.values() for value in curve]
    return max(values) if values else 0.0


def set_auto_axes(ax, frames: Sequence[int], ymax_value: float):
    xmax = rounded_axis_max(max(frames) if frames else 0, base=10, minimum=10)
    ymax = rounded_axis_max(ymax_value, base=10, minimum=10)
    ax.set_xlim(0, xmax)
    ax.set_ylim(0, ymax)


def parse_exclusions(values: Sequence[str]) -> Dict[str, set[str]]:
    exclusions = {key: set(value) for key, value in DEFAULT_EXCLUDE.items()}
    for value in values:
        if ":" not in value:
            raise ValueError("--exclude format is REVISION:LABEL[,LABEL...]")
        revision, labels = value.split(":", 1)
        exclusions.setdefault(revision, set()).update(
            item.strip().lower() for item in labels.split(",") if item.strip()
        )
    return exclusions


def plot_revisions(args):
    import matplotlib.pyplot as plt

    results_dir = Path(args.results_dir).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    exclusions = parse_exclusions(args.exclude)

    revision_folders = []
    for revision in args.revisions:
        folder = results_dir / revision
        if folder.exists():
            revision_folders.append(folder)
        else:
            print(f"skip missing revision folder: {folder}")

    plotted = []
    for folder in revision_folders:
        rev_name = folder.name
        short_label = revision_label(folder)
        exclude = exclusions.get(rev_name, set()) | exclusions.get(short_label, set())
        series = load_revision(folder, exclude)
        if len(series) < 2:
            print(f"skip {rev_name}: need at least 2 CSV series, found {len(series)}")
            continue
        frames, errors, skews = frame_errors_ms(series, args.metric, args.max_frames)
        print(summarize(rev_name, skews))

        fig, ax = plt.subplots(figsize=(9, 4.8), dpi=args.dpi)
        for label in sorted(errors, key=natural_key):
            ax.plot(frames, errors[label], linewidth=1.0, label=label)
        ax.set_title(f"Frame-wise synchronization error ({short_label})")
        ax.set_xlabel("frame index")
        ax.set_ylabel("timestamp error [ms]")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right", fontsize=9)
        set_auto_axes(ax, frames, max_error_value(errors))
        fig.tight_layout()
        out_path = output_dir / f"{short_label}_frame_error_{args.metric}.png"
        fig.savefig(out_path)
        plt.close(fig)
        std_frames, stddevs = frame_stddev_ms(
            series, args.max_frames, args.stddev_relative_zero
        )
        if args.frame_stddev_each and stddevs:
            fig, ax = plt.subplots(figsize=(9, 4.8), dpi=args.dpi)
            ax.plot(std_frames, stddevs, linewidth=1.4, label=short_label)
            ax.set_title(f"Frame-wise timestamp standard deviation ({short_label})")
            ax.set_xlabel("frame index")
            ax.set_ylabel("standard deviation [ms]")
            ax.grid(True, alpha=0.25)
            ax.legend(loc="upper right", fontsize=9)
            set_auto_axes(ax, std_frames, max(stddevs))
            fig.tight_layout()
            suffix = "relative_zero" if args.stddev_relative_zero else "absolute"
            out_path = output_dir / f"{short_label}_frame_stddev_{suffix}.png"
            fig.savefig(out_path)
            plt.close(fig)
        if args.robot_stddev_each:
            dev_frames, robot_devs, robot_stddevs = frame_robot_stddev_component_ms(
                series, args.max_frames, args.stddev_relative_zero
            )
            if robot_devs:
                fig, ax = plt.subplots(figsize=(9, 4.8), dpi=args.dpi)
                for label in sorted(robot_devs, key=natural_key):
                    ax.plot(dev_frames, robot_devs[label], linewidth=1.0, label=label)
                if args.overlay_frame_stddev:
                    ax.plot(
                        dev_frames, robot_stddevs,
                        linewidth=1.8, linestyle="--", color="black",
                        label="frame stddev",
                    )
                ax.set_title(f"Robot-wise standard-deviation component ({short_label})")
                ax.set_xlabel("frame index")
                ax.set_ylabel("standard-deviation component [ms]")
                ax.grid(True, alpha=0.25)
                ax.legend(loc="upper right", fontsize=9)
                set_auto_axes(ax, dev_frames, max_error_value(robot_devs))
                fig.tight_layout()
                suffix = "relative_zero" if args.stddev_relative_zero else "absolute"
                out_path = output_dir / f"{short_label}_robot_stddev_component_{suffix}.png"
                fig.savefig(out_path)
                plt.close(fig)
        plotted.append((short_label, frames, errors, skews, std_frames, stddevs))

    if args.combined and plotted:
        rows = len(plotted)
        fig, axes = plt.subplots(
            rows, 1, figsize=(9, max(3.2, 2.8 * rows)), dpi=args.dpi,
            sharex=False
        )
        if rows == 1:
            axes = [axes]
        for ax, (short_label, frames, errors, skews, std_frames, stddevs) in zip(axes, plotted):
            for label in sorted(errors, key=natural_key):
                ax.plot(frames, errors[label], linewidth=0.9, label=label)
            ax.set_title(f"Frame-wise synchronization error ({short_label})")
            ax.set_xlabel("frame index")
            ax.set_ylabel("error [ms]")
            ax.grid(True, alpha=0.25)
            ax.legend(loc="upper right", fontsize=8)
            set_auto_axes(ax, frames, max_error_value(errors))
        fig.tight_layout()
        out_path = output_dir / f"all_revisions_frame_error_{args.metric}.png"
        fig.savefig(out_path)
        plt.close(fig)
        print(f"saved combined figure: {out_path}")

    if args.revision_mean and plotted:
        fig, ax = plt.subplots(figsize=(9, 4.8), dpi=args.dpi)
        all_mean_values = []
        all_frames = []
        for short_label, frames, errors, skews, std_frames, stddevs in plotted:
            mean_curve = []
            labels = list(errors)
            for frame_idx in range(len(frames)):
                values = [errors[label][frame_idx] for label in labels]
                mean_curve.append(sum(values) / len(values))
            all_mean_values.extend(mean_curve)
            all_frames.extend(frames)
            ax.plot(frames, mean_curve, linewidth=1.4, label=short_label)
        ax.set_title(f"Mean frame-wise synchronization error by revision")
        ax.set_xlabel("frame index")
        ax.set_ylabel("mean timestamp error [ms]")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right", fontsize=8, ncol=2)
        set_auto_axes(ax, all_frames, max(all_mean_values) if all_mean_values else 0.0)
        fig.tight_layout()
        out_path = output_dir / f"revision_mean_frame_error_{args.metric}.png"
        fig.savefig(out_path)
        plt.close(fig)
        print(f"saved revision-mean figure: {out_path}")

    if args.frame_stddev and plotted:
        fig, ax = plt.subplots(figsize=(9, 4.8), dpi=args.dpi)
        all_std_values = []
        all_std_frames = []
        for short_label, frames, errors, skews, std_frames, stddevs in plotted:
            if not stddevs:
                continue
            all_std_values.extend(stddevs)
            all_std_frames.extend(std_frames)
            ax.plot(std_frames, stddevs, linewidth=1.4, label=short_label)
        ax.set_title("Frame-wise timestamp standard deviation by revision")
        ax.set_xlabel("frame index")
        ax.set_ylabel("standard deviation [ms]")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right", fontsize=8, ncol=2)
        set_auto_axes(
            ax,
            all_std_frames,
            max(all_std_values) if all_std_values else 0.0,
        )
        fig.tight_layout()
        suffix = "relative_zero" if args.stddev_relative_zero else "absolute"
        out_path = output_dir / f"revision_frame_stddev_{suffix}.png"
        fig.savefig(out_path)
        plt.close(fig)
        print(f"saved frame-stddev figure: {out_path}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", default=str(DEFAULT_RESULTS_DIR))
    parser.add_argument("--output-dir", default="revision_error_plots")
    parser.add_argument("--revisions", nargs="+", default=DEFAULT_REVISIONS)
    parser.add_argument(
        "--metric",
        choices=("median_abs", "first_abs", "relative_zero"),
        default="median_abs",
        help=(
            "median_abs: per-camera absolute error from frame-wise median; "
            "first_abs: error from first CSV in the folder; "
            "relative_zero: remove first-frame offsets and plot drift"
        ),
    )
    parser.add_argument("--exclude", action="append", default=[],
                        help="exclude series, e.g. r11_20260702:vcrobot2")
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--dpi", type=int, default=180)
    parser.add_argument("--combined", action="store_true",
                        help="also save one tall figure containing all revisions")
    parser.add_argument("--revision-mean", action="store_true",
                        help=("save one plot where each line is the per-frame "
                              "mean error of one revision"))
    parser.add_argument("--frame-stddev", action="store_true",
                        help=("save one plot where each line is the per-frame "
                              "timestamp standard deviation of one revision"))
    parser.add_argument("--frame-stddev-each", action="store_true",
                        help="save one per-frame standard-deviation plot per revision")
    parser.add_argument("--robot-stddev-each", action="store_true",
                        help=("save one plot per revision with one line per robot: "
                              "per-frame sample-standard-deviation component"))
    parser.add_argument("--overlay-frame-stddev", action="store_true",
                        help=("with --robot-stddev-each, overlay the aggregate "
                              "frame standard deviation as a dashed black line"))
    parser.add_argument("--stddev-relative-zero", action="store_true",
                        help=("for --frame-stddev, remove each camera's first-frame "
                              "timestamp before computing standard deviation"))
    return parser.parse_args()


def main():
    args = parse_args()
    plot_revisions(args)


if __name__ == "__main__":
    main()
