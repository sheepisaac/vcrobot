#!/usr/bin/env python3
"""Synchronized x/y/z/t RoArm controller for multiple robots."""

import argparse
import json
import math

from master_arm import Master, DEFAULT_PORT, discover_slaves


def make_command(values, speed):
    if len(values) != 4:
        raise ValueError("enter exactly four values: x y z t")
    x, y, z, t = (float(value) for value in values)
    if not all(math.isfinite(value) for value in (x, y, z, t, speed)):
        raise ValueError("position and speed values must be finite numbers")
    if speed <= 0:
        raise ValueError("speed must be positive")
    return json.dumps(
        {"T": 104, "x": x, "y": y, "z": z, "t": t, "spd": speed},
        separators=(",", ":"),
    )


def make_verification(command, args):
    if not args.verify_position:
        return None
    target = json.loads(command)
    return {
        "target": {key: target[key] for key in ("x", "y", "z", "t")},
        "ack_timeout": args.ack_timeout,
        "timeout": args.verify_timeout,
        "feedback_start_delay": args.feedback_start_delay,
        "xyz_tolerance": args.xyz_tolerance,
        "t_tolerance": args.t_tolerance,
        "stable_samples": args.stable_samples,
        "require_command_ack": args.require_command_ack,
    }


def dispatch_position(master, command, lead, verification):
    command_spec = ((0.0, "position", command, verification)
                    if verification is not None else
                    (0.0, "position", command))
    results = master.dispatch([command_spec], lead)
    if verification is None:
        return
    failures = [item for item in results
                if not (item.get("verification") or {}).get("success")]
    if failures:
        detail = ", ".join(
            f"{item['robot'].endpoint}="
            f"{(item.get('verification') or {}).get('reason', 'no_verification')}"
            for item in failures
        )
        raise RuntimeError(f"arm position verification failed: {detail}")


def interactive(master, lead, speed, verification_args):
    print("Input x y z t (example: 100 0 480 4.1)")
    print("Commands: sync, quit")
    while True:
        try:
            text = input("arm-simple-master> ").strip()
            if not text:
                continue
            if text.lower() in ("quit", "exit"):
                return
            if text.lower() == "sync":
                sync_clock = "tai" if master.clock_mode == "ptp" else "monotonic"
                for robot in master.robots:
                    robot.synchronize(master.sync_samples, sync_clock)
                    print(f"{robot.endpoint}: RTT={robot.delay_ns / 1e6:.3f}ms "
                          f"offset={robot.offset_ns / 1e6:+.3f}ms")
                continue
            command = make_command(text.split(), speed)
            dispatch_position(master, command, lead,
                              make_verification(command, verification_args))
        except (ValueError, RuntimeError, OSError, json.JSONDecodeError) as exc:
            print(f"Error: {exc}")
        except KeyboardInterrupt:
            print()
            return


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", nargs="+",
                        help="explicit robot hostnames/IPs; disables discovery")
    parser.add_argument("--discover-subnet", default="192.168.10.0/24")
    parser.add_argument("--discovery-port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--discovery-timeout", type=float, default=0.75)
    parser.add_argument("--discovery-workers", type=int, default=32)
    parser.add_argument("--expected", type=int, default=2)
    parser.add_argument("--lead", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--sync-samples", type=int, default=15)
    parser.add_argument("--spd", type=float, default=0.75)
    parser.add_argument("--ack-timeout", type=float, default=0.3,
                        help="seconds to wait for a direct command response")
    parser.add_argument("--verify-position", action="store_true",
                        help=("poll arm feedback after dispatch and fail when the target "
                              "pose is not reached; off by default because polling can "
                              "disturb motion on some RoArm controllers"))
    parser.add_argument("--verify-timeout", type=float, default=8.0,
                        help="seconds to wait for target position feedback")
    parser.add_argument("--feedback-start-delay", type=float, default=2.0,
                        help=("seconds to wait after dispatch before feedback polling "
                              "when --verify-position is enabled"))
    parser.add_argument("--xyz-tolerance", type=float, default=8.0,
                        help="Cartesian target tolerance in controller units (normally mm)")
    parser.add_argument("--t-tolerance", type=float, default=0.15,
                        help="tool-angle target tolerance")
    parser.add_argument("--stable-samples", type=int, default=2,
                        help="consecutive in-tolerance feedback samples required")
    parser.add_argument("--require-command-ack", action="store_true",
                        help="fail unless the T=104 command itself returns JSON")
    parser.add_argument("--clock-mode", choices=("estimated", "ptp"),
                        default="estimated",
                        help="estimated TCP clock offsets or strict PTP/CLOCK_TAI")
    parser.add_argument("--ptp-max-offset-ms", type=float, default=1.0,
                        help="abort PTP mode when measured TAI offset exceeds this")
    parser.add_argument("--position", nargs=4, metavar=("X", "Y", "Z", "T"),
                        help="send one position and exit")
    args = parser.parse_args()
    if args.lead < 0.2:
        parser.error("--lead must be at least 0.2 seconds")
    if args.sync_samples < 3:
        parser.error("--sync-samples must be at least 3")
    if args.expected < 1:
        parser.error("--expected must be at least 1")
    if args.discovery_timeout <= 0 or args.discovery_workers < 1:
        parser.error("discovery timeout/workers must be positive")
    if not math.isfinite(args.spd) or args.spd <= 0:
        parser.error("--spd must be a positive finite number")
    if (args.ack_timeout <= 0 or args.verify_timeout <= 0 or
            args.feedback_start_delay < 0 or
            args.xyz_tolerance < 0 or args.t_tolerance < 0):
        parser.error("verification timeouts must be positive and tolerances nonnegative")
    if args.stable_samples < 1:
        parser.error("--stable-samples must be at least 1")
    if args.ptp_max_offset_ms <= 0:
        parser.error("--ptp-max-offset-ms must be positive")
    return args


def resolve_endpoints(args):
    if args.robots:
        return args.robots
    endpoints = discover_slaves(
        args.discover_subnet,
        args.discovery_port,
        args.discovery_timeout,
        args.discovery_workers,
        expected=args.expected,
    )
    if len(endpoints) != args.expected:
        raise SystemExit(
            f"Discovery found {len(endpoints)} slave(s), expected {args.expected}.\n"
            "Start slave_armSimple.py on every robot or use --robots IP [IP ...]."
        )
    return endpoints


def main():
    args = parse_args()
    try:
        endpoints = resolve_endpoints(args)
        master = Master(endpoints, args.timeout, args.sync_samples,
                        args.clock_mode, args.ptp_max_offset_ms)
    except (OSError, RuntimeError, ConnectionError, ValueError) as exc:
        raise SystemExit(f"Could not initialize arm slaves: {exc}")
    try:
        if args.position:
            command = make_command(args.position, args.spd)
            dispatch_position(master, command, args.lead,
                              make_verification(command, args))
        else:
            interactive(master, args.lead, args.spd, args)
    finally:
        master.close()


if __name__ == "__main__":
    main()
