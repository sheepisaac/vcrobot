#!/usr/bin/env python3
"""Server-PC master for synchronized RealSense single-image capture."""

import argparse

from realsense_master_sync import (
    add_common_master_args,
    run_capture,
    validate_common_args,
)


DEFAULT_PORT = 50310


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_master_args(parser, DEFAULT_PORT)
    parser.add_argument("--frame-number", type=int, required=True)
    parser.add_argument("--sync-mode", choices=("timestamp", "gate"),
                        default="timestamp")
    parser.add_argument("--collect-before-ms", type=float, default=80.0)
    parser.add_argument("--collect-after-ms", type=float, default=80.0)
    args = parser.parse_args()
    validate_common_args(parser, args)
    return args


def main():
    args = parse_args()
    command = {
        "action": "image",
        "frame_number": args.frame_number,
        "width": args.width,
        "height": args.height,
        "output_dir": args.output_dir,
        "sync_mode": args.sync_mode,
        "collect_before_ms": args.collect_before_ms,
        "collect_after_ms": args.collect_after_ms,
    }
    run_capture(args, command, "image")


if __name__ == "__main__":
    main()
