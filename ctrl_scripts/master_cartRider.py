#!/usr/bin/env python3
"""Synchronized TCP master based on ctrl_cartRider.py."""

import argparse
import curses
import json
import threading
import time

from drive_master_sync import DriveMaster, STOP_COMMAND, resolve_drive_endpoints


PORT = 50221


def direction_command(direction, speed):
    mapping = {
        "w": (speed, speed), "s": (-speed, -speed),
        "a": (-speed, speed), "d": (speed, -speed),
    }
    if direction not in mapping:
        raise ValueError("direction must be w, a, s, or d")
    left, right = mapping[direction]
    return json.dumps({"T": 1, "L": left, "R": right}, separators=(",", ":"))


class KeyCommandSender:
    def __init__(self, master, lead, speed):
        self.master, self.lead, self.speed = master, lead, speed
        self.cv = threading.Condition()
        self.version = self.processed_version = 0
        self.direction = None
        self.status = "ready"
        self.stopping = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def request(self, direction):
        with self.cv:
            if direction == self.direction:
                return self.version
            self.direction = direction
            self.version += 1
            self.cv.notify_all()
            return self.version

    def snapshot(self):
        with self.cv:
            return self.direction, self.status

    def _run(self):
        seen = 0
        while True:
            with self.cv:
                while seen == self.version and not self.stopping:
                    self.cv.wait()
                if self.stopping and seen == self.version:
                    return
                direction, version = self.direction, self.version
                self.status = f"scheduling {direction or 'stop'}"
            try:
                command = (STOP_COMMAND if direction is None else
                           direction_command(direction, self.speed))
                results = self.master.dispatch(
                    [(0.0, direction or "stop", command)], self.lead, report=False)
                skew = 0.0
                if len(results) > 1:
                    actual = [item["actual_master_ns"] for item in results]
                    skew = (max(actual) - min(actual)) / 1e6
                state = f"active {direction}" if direction else "stopped"
                status = f"{state}; UART skew={skew:.3f}ms"
            except Exception as exc:
                status = f"ERROR: {exc}"
            with self.cv:
                seen = version
                self.processed_version = version
                self.status = status
                self.cv.notify_all()

    def close(self):
        target = self.request(None)
        deadline = time.monotonic() + max(2.0, self.lead + 1.0)
        with self.cv:
            while self.processed_version < target and time.monotonic() < deadline:
                self.cv.wait(0.05)
            self.stopping = True
            self.cv.notify_all()
        self.thread.join(timeout=1.0)


def curses_control(stdscr, master, lead, speed):
    sender = KeyCommandSender(master, lead, speed)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    hold_timeout = 1.0
    last_key_time = time.monotonic()
    active_direction = None
    try:
        while True:
            key = stdscr.getch()
            now = time.monotonic()
            if key in (ord("w"), ord("a"), ord("s"), ord("d")):
                direction = chr(key)
                last_key_time = now
                if direction != active_direction:
                    active_direction = direction
                    sender.request(direction)
            elif key == ord("q"):
                sender.request(None)
                break
            elif key == -1:
                if active_direction is not None and now - last_key_time > hold_timeout:
                    active_direction = None
                    sender.request(None)
            else:
                active_direction = None
                sender.request(None)

            requested, status = sender.snapshot()
            stdscr.erase()
            stdscr.addstr(0, 0, "WASD drive | release/idle 1s: stop | q: quit\n")
            stdscr.addstr(1, 0, f"speed={speed:.3f} lead={lead:.3f}s\n")
            stdscr.addstr(2, 0, f"requested={requested or 'stop'}\n")
            stdscr.addstr(3, 0, f"status={status}"[:max(1, curses.COLS - 1)])
            stdscr.refresh()
            time.sleep(0.02)
    finally:
        sender.close()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", nargs="+")
    parser.add_argument("--discover-subnet", default="192.168.10.0/24")
    parser.add_argument("--discovery-timeout", type=float, default=0.75)
    parser.add_argument("--discovery-workers", type=int, default=32)
    parser.add_argument("--expected", type=int, default=2)
    parser.add_argument("--lead", type=float, default=0.25)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--sync-samples", type=int, default=15)
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--clock-mode", choices=("estimated", "ptp"),
                        default="estimated")
    parser.add_argument("--ptp-max-offset-ms", type=float, default=1.0)
    args = parser.parse_args()
    if not (0 < args.speed <= 1.0):
        parser.error("--speed must be in (0, 1]")
    return args


def main():
    args = parse_args()
    endpoints = resolve_drive_endpoints(
        args.robots, args.discover_subnet, PORT, args.discovery_timeout,
        args.discovery_workers, args.expected)
    master = DriveMaster(endpoints, args.timeout, args.sync_samples,
                         args.clock_mode, args.ptp_max_offset_ms)
    try:
        curses.wrapper(curses_control, master, args.lead, args.speed)
    finally:
        master.close()


if __name__ == "__main__":
    main()
