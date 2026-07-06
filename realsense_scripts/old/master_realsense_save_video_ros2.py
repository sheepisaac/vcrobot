#!/usr/bin/env python3
"""Server-PC master for synchronized RealSense video capture."""

import argparse

from realsense_master_sync import (
    add_common_master_args,
    run_capture,
    validate_common_args,
)


DEFAULT_PORT = 50311
DEFAULT_EXTRA_FRAMES = 30
DEFAULT_EXTRA_RATIO = 0.20


def auto_capture_frames(target_frames):
    extra = max(DEFAULT_EXTRA_FRAMES, int(round(target_frames * DEFAULT_EXTRA_RATIO)))
    return target_frames + extra


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_master_args(parser, DEFAULT_PORT)
    parser.add_argument("--target-frames", type=int, required=True,
                        help="final synchronized frame count M")
    parser.add_argument("--frame-count", type=int,
                        help=argparse.SUPPRESS)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--sample-mode", choices=("every_frame", "timed"),
                        default="every_frame",
                        help=("every_frame writes each incoming camera frame; "
                              "timed samples using --fps intervals"))
    parser.add_argument("--sync-mode", choices=("timestamp", "gate"),
                        default="timestamp")
    parser.add_argument("--collect-before-ms", type=float, default=80.0)
    parser.add_argument("--collect-after-ms", type=float, default=80.0)
    parser.add_argument("--filename",
                        help="optional YUV filename stored under output_dir/YYYYMMDD/")
    args = parser.parse_args()
    validate_common_args(parser, args)
    if args.target_frames < 1:
        parser.error("--target-frames must be positive")
    args.capture_frames = auto_capture_frames(args.target_frames)
    if args.fps <= 0:
        parser.error("--fps must be positive")
    return args


def main():
    args = parse_args()
    print(
        f"Target frames={args.target_frames}; auto capture frames="
        f"{args.capture_frames} "
        f"(extra={args.capture_frames - args.target_frames})"
    )
    command = {
        "action": "video",
        "frame_count": args.capture_frames,
        "target_frames": args.target_frames,
        "capture_frames": args.capture_frames,
        "width": args.width,
        "height": args.height,
        "fps": args.fps,
        "sample_mode": args.sample_mode,
        "output_dir": args.output_dir,
        "sync_mode": args.sync_mode,
        "collect_before_ms": args.collect_before_ms,
        "collect_after_ms": args.collect_after_ms,
    }
    if args.filename:
        command["filename"] = args.filename
    run_capture(args, command, "video")


if __name__ == "__main__":
    main()
