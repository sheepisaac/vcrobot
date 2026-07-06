#!/usr/bin/env python3
"""Master-PC helpers for synchronized UGV drive controllers."""

import threading

from drive_sync import STOP_COMMAND
from master_arm import Master, discover_slaves


class DriveMaster(Master):
    def __init__(self, endpoints, timeout, samples, clock_mode="estimated",
                 ptp_max_offset_ms=1.0):
        super().__init__(endpoints, timeout, samples, clock_mode, ptp_max_offset_ms)
        self.heartbeat_stop = threading.Event()
        self.heartbeat_thread = threading.Thread(target=self._heartbeat, daemon=True)
        self.heartbeat_thread.start()

    def _heartbeat(self):
        while not self.heartbeat_stop.wait(0.2):
            for robot in self.robots:
                try:
                    robot.send({"type": "heartbeat"})
                except Exception:
                    return

    def close(self):
        if hasattr(self, "heartbeat_stop"):
            self.heartbeat_stop.set()
        if hasattr(self, "heartbeat_thread"):
            self.heartbeat_thread.join(timeout=1.0)
        super().close()


def resolve_drive_endpoints(robots, subnet, port, timeout, workers, expected):
    if robots:
        return [value if ":" in value else f"{value}:{port}" for value in robots]
    found = discover_slaves(subnet, port, timeout, workers, expected=expected)
    if len(found) != expected:
        raise SystemExit(f"Discovery found {len(found)} drive slave(s), expected {expected}")
    return [f"{value}:{port}" for value in found]
