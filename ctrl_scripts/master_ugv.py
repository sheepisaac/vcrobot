#!/usr/bin/env python3
"""Synchronized TCP master based on ctrl_ugv.py."""

import argparse
import json

from drive_master_sync import DriveMaster, STOP_COMMAND, resolve_drive_endpoints


PORT = 50220


def go_command(left, right):
    left, right = float(left), float(right)
    if not (-1.0 <= left <= 1.0 and -1.0 <= right <= 1.0):
        raise ValueError("L and R must be between -1.0 and 1.0")
    return json.dumps({"T": 1, "L": left, "R": right}, separators=(",", ":"))


def interactive(master, lead):
    print("Commands: go L R | stop | run SECONDS L R | sync | quit")
    while True:
        try:
            parts = input("ugv-master> ").strip().lower().split()
            if not parts:
                continue
            if parts[0] in ("quit", "exit"):
                master.dispatch([(0.0, "stop", STOP_COMMAND)], lead)
                return
            if parts[0] == "sync":
                clock = "tai" if master.clock_mode == "ptp" else "monotonic"
                for robot in master.robots:
                    robot.synchronize(master.sync_samples, clock)
                    print(f"{robot.endpoint}: RTT={robot.delay_ns/1e6:.3f}ms "
                          f"offset={robot.offset_ns/1e6:+.3f}ms")
            elif parts[0] == "stop" and len(parts) == 1:
                master.dispatch([(0.0, "stop", STOP_COMMAND)], lead)
            elif parts[0] == "go" and len(parts) == 3:
                master.dispatch([(0.0, "go", go_command(parts[1], parts[2]))], lead)
            elif parts[0] == "run" and len(parts) == 4:
                duration = float(parts[1])
                if duration <= 0:
                    raise ValueError("duration must be positive")
                master.dispatch([
                    (0.0, "start", go_command(parts[2], parts[3])),
                    (duration, "stop", STOP_COMMAND),
                ], lead)
            else:
                raise ValueError("use: go L R | stop | run SECONDS L R")
        except (ValueError, RuntimeError, OSError) as exc:
            print(f"Error: {exc}")
        except KeyboardInterrupt:
            print()
            try:
                master.dispatch([(0.0, "stop", STOP_COMMAND)], lead)
            except Exception:
                pass
            return


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", nargs="+")
    parser.add_argument("--discover-subnet", default="192.168.10.0/24")
    parser.add_argument("--discovery-timeout", type=float, default=0.75)
    parser.add_argument("--discovery-workers", type=int, default=32)
    parser.add_argument("--expected", type=int, default=2)
    parser.add_argument("--lead", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--sync-samples", type=int, default=15)
    parser.add_argument("--clock-mode", choices=("estimated", "ptp"),
                        default="estimated")
    parser.add_argument("--ptp-max-offset-ms", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    endpoints = resolve_drive_endpoints(
        args.robots, args.discover_subnet, PORT, args.discovery_timeout,
        args.discovery_workers, args.expected)
    master = DriveMaster(endpoints, args.timeout, args.sync_samples,
                         args.clock_mode, args.ptp_max_offset_ms)
    try:
        interactive(master, args.lead)
    finally:
        master.close()


if __name__ == "__main__":
    main()
