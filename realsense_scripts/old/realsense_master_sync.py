#!/usr/bin/env python3
"""Master helpers for synchronized RealSense capture slaves."""

import argparse
import json
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
CTRL_DIR = SCRIPT_DIR.parent / "ctrl_scripts"
if str(CTRL_DIR) not in sys.path:
    sys.path.insert(0, str(CTRL_DIR))

from master_arm import Master, discover_slaves  # noqa: E402


def endpoint_list(robots, subnet, port, timeout, workers, expected):
    if robots:
        return [value if ":" in value else f"{value}:{port}" for value in robots]
    found = discover_slaves(subnet, port, timeout, workers, expected=expected)
    if len(found) != expected:
        raise SystemExit(
            f"Discovery found {len(found)} RealSense slave(s), expected {expected}."
        )
    return [f"{value}:{port}" for value in found]


def add_common_master_args(parser, default_port):
    parser.add_argument("--robots", nargs="+",
                        help="explicit robot hostnames/IPs; disables discovery")
    parser.add_argument("--discover-subnet", default="192.168.10.0/24")
    parser.add_argument("--discovery-port", type=int, default=default_port)
    parser.add_argument("--discovery-timeout", type=float, default=0.75)
    parser.add_argument("--discovery-workers", type=int, default=32)
    parser.add_argument("--expected", type=int, default=2)
    parser.add_argument("--lead", type=float, default=1.0,
                        help="seconds between READY phase and capture gate opening")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--sync-samples", type=int, default=25)
    parser.add_argument("--clock-mode", choices=("estimated", "ptp"),
                        default="estimated",
                        help="Use ptp only when every host has CLOCK_TAI synchronized")
    parser.add_argument("--ptp-max-offset-ms", type=float, default=1.0)
    parser.add_argument("--output-dir", default="./Results")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    return parser


def run_capture(args, command, label):
    endpoints = endpoint_list(
        args.robots, args.discover_subnet, args.discovery_port,
        args.discovery_timeout, args.discovery_workers, args.expected
    )
    master = Master(
        endpoints, args.timeout, args.sync_samples,
        args.clock_mode, args.ptp_max_offset_ms
    )
    try:
        results = master.dispatch(
            [(0.0, label, json.dumps(command, separators=(",", ":")))],
            args.lead,
            report=False,
        )
        print_camera_report(results, args.clock_mode)
    finally:
        master.close()


def print_camera_report(results, clock_mode):
    activation = [item["actual_master_ns"] for item in results]
    activation_skew_ms = (max(activation) - min(activation)) / 1e6 if activation else 0.0
    frame_times = []
    print("RealSense execution report:")
    print(f"  capture gate skew={activation_skew_ms:.3f} ms")
    for item in results:
        verification = item.get("verification") or {}
        capture = verification.get("capture") or {}
        camera_status = verification.get("camera_status") or {}
        saved_ns = capture.get("saved_frame_clock_ns")
        if saved_ns is not None:
            if clock_mode != "ptp":
                saved_master_ns = int(saved_ns) - item["robot"].offset_ns
            else:
                saved_master_ns = int(saved_ns)
            frame_times.append(saved_master_ns)
        print(
            f"  {item['robot'].endpoint}: "
            f"activation_late={item['late_ns'] / 1e6:+.3f}ms "
            f"success={verification.get('success')} "
            f"reason={verification.get('reason')} "
            f"file={(capture or {}).get('filename')} "
            f"csv={(capture or {}).get('csv_filename')} "
            f"sync_mode={(capture or {}).get('sync_mode')} "
            f"timestamp_error_ms={(capture or {}).get('timestamp_error_ms')} "
            f"candidate_count={(capture or {}).get('candidate_count')} "
            f"frame_late_from_gate={(capture or {}).get('late_from_accept_ms')}ms "
            f"ros_stamp_ns={(capture or {}).get('ros_stamp_ns')} "
            f"encoding={(capture or {}).get('encoding') or camera_status.get('last_encoding')} "
            f"frames_seen={camera_status.get('frames_seen')} "
            f"last_frame_age_ms={camera_status.get('last_callback_age_ms')} "
            f"warmup_ready={camera_status.get('warmup_ready')} "
            f"warmup_elapsed_ms={camera_status.get('warmup_elapsed_ms')}"
        )
    if len(frame_times) >= 2:
        print(f"  first saved/callback frame skew={(max(frame_times) - min(frame_times)) / 1e6:.3f} ms")
    print("  note: 10ms 이하 판정은 'first saved/callback frame skew'를 보세요.")


def validate_common_args(parser, args):
    if args.lead < 0.2:
        parser.error("--lead must be at least 0.2 seconds")
    if args.sync_samples < 3:
        parser.error("--sync-samples must be at least 3")
    if args.expected < 1:
        parser.error("--expected must be at least 1")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width/--height must be positive")
