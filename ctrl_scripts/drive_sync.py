#!/usr/bin/env python3
"""Shared synchronized TCP runtime for UGV drive controllers."""

import argparse
import heapq
import json
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

PROTOCOL_VERSION = 2
# Ordinary drive stop: leave the independently controlled arm energized.
STOP_COMMAND = '{"T":1,"L":0.0,"R":0.0}'
# Preserve the existing global safety-stop behavior on disconnect/fault/shutdown.
EMERGENCY_STOP_COMMAND = '{"T":0}'
TAI_CLOCK_ID = getattr(time, "CLOCK_TAI", 11)


def tai_available():
    if not hasattr(time, "clock_gettime_ns"):
        return False
    try:
        time.clock_gettime_ns(TAI_CLOCK_ID)
        return True
    except (OSError, ValueError):
        return False


def clock_now_ns(mode):
    if mode == "monotonic":
        return time.monotonic_ns()
    if mode == "tai" and tai_available():
        return time.clock_gettime_ns(TAI_CLOCK_ID)
    raise ValueError(f"unsupported clock mode: {mode}")


def normalize_drive_command(command):
    value = json.loads(command)
    if not isinstance(value, dict) or value.get("T") not in (0, 1):
        raise ValueError("drive command must be T=0 or T=1 JSON")
    if value["T"] == 0:
        # Older drive masters use T=0 for an ordinary stop key/command.
        return STOP_COMMAND, 0.0, 0.0
    left, right = float(value["L"]), float(value["R"])
    if not (-1.0 <= left <= 1.0 and -1.0 <= right <= 1.0):
        raise ValueError("L and R must be between -1.0 and 1.0")
    return json.dumps({"T": 1, "L": left, "R": right},
                      separators=(",", ":")), left, right


def move_toward(current, target, step):
    if current < target:
        return min(current + step, target)
    if current > target:
        return max(current - step, target)
    return current


class DriveSerial:
    def __init__(self, mode, port, baudrate, timeout, dry_run,
                 keepalive, ramp_step, reverse_deadtime):
        self.mode = mode
        self.lock = threading.RLock()
        self.state_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.keepalive = keepalive
        self.ramp_step = ramp_step
        self.reverse_deadtime = reverse_deadtime
        self.current_left = self.current_right = 0.0
        self.target_left = self.target_right = 0.0
        self.current_command = STOP_COMMAND
        self.last_write = 0.0
        self.reverse_allowed_at = time.monotonic()
        self.ramp_interval = 0.02
        self.next_ramp_at = time.monotonic()
        self.dry_run = dry_run
        if dry_run:
            self.serial = None
            print("DRY-RUN: drive serial output disabled", flush=True)
        else:
            import serial
            self.serial = serial.Serial(port, baudrate=baudrate, timeout=timeout,
                                        exclusive=True)
            print(f"Drive serial opened: {port} @ {baudrate} mode={mode}", flush=True)
        self._write(STOP_COMMAND)
        print("Ordinary drive stop: T=1 L=0 R=0 (arm torque unchanged); "
              "disconnect/fault/shutdown still uses global T=0", flush=True)
        self.worker = threading.Thread(target=self._background, daemon=True)
        self.worker.start()

    @staticmethod
    def _format(left, right):
        if left == 0.0 and right == 0.0:
            return STOP_COMMAND
        return json.dumps({"T": 1, "L": round(left, 3), "R": round(right, 3)},
                          separators=(",", ":"))

    def _write(self, command):
        data = command.rstrip("\r\n").encode() + b"\n"
        with self.lock:
            actual_ns = time.monotonic_ns()
            if self.serial is not None:
                self.serial.write(data)
                self.serial.flush()
            self.last_write = time.monotonic()
        return actual_ns

    def apply_at(self, command, execute_at_ns, clock_mode):
        command, left, right = normalize_drive_command(command)
        with self.lock:
            while True:
                remaining = execute_at_ns - time.monotonic_ns()
                if remaining <= 0:
                    break
                if remaining > 1_000_000:
                    time.sleep((remaining - 500_000) / 1e9)
            actual_mono_ns = time.monotonic_ns()
            actual_clock_ns = (actual_mono_ns if clock_mode == "monotonic"
                               else clock_now_ns("tai"))
            with self.state_lock:
                if self.mode == "cart" and command != STOP_COMMAND:
                    reversing = (self.current_left * left < 0 or
                                 self.current_right * right < 0)
                    self.target_left, self.target_right = left, right
                    if reversing:
                        self.current_left = self.current_right = 0.0
                        self.reverse_allowed_at = time.monotonic() + self.reverse_deadtime
                        self.next_ramp_at = self.reverse_allowed_at
                    else:
                        self.current_left = move_toward(self.current_left, left,
                                                        self.ramp_step)
                        self.current_right = move_toward(self.current_right, right,
                                                         self.ramp_step)
                        self.next_ramp_at = execute_at_ns / 1e9 + self.ramp_interval
                    command = self._format(self.current_left, self.current_right)
                else:
                    self.target_left, self.target_right = left, right
                    self.current_left, self.current_right = left, right
                self.current_command = command
            if self.serial is not None:
                self.serial.write(command.encode() + b"\n")
                self.serial.flush()
            self.last_write = time.monotonic()
        return actual_clock_ns

    def emergency_stop(self, reason):
        # Keep global T=0 for safety stops. Serialize with the ramp writer so
        # a previously selected movement packet cannot be sent after the stop.
        with self.lock:
            with self.state_lock:
                self.target_left = self.target_right = 0.0
                self.current_left = self.current_right = 0.0
                self.current_command = EMERGENCY_STOP_COMMAND
                self.next_ramp_at = time.monotonic()
            try:
                self._write(EMERGENCY_STOP_COMMAND)
                print(f"EMERGENCY STOP: {reason}", flush=True)
            except Exception as exc:
                print(f"EMERGENCY STOP write failed: {exc}", flush=True)

    def _background(self):
        previous = STOP_COMMAND
        while not self.stop_event.wait(0.002):
            try:
                previous = self._background_step(previous)
            except Exception as exc:
                print(f"Drive writer stopped: {exc}", flush=True)
                self.stop_event.set()

    def _background_step(self, previous):
        with self.lock:
            with self.state_lock:
                now = time.monotonic()
                if (self.current_command != EMERGENCY_STOP_COMMAND and
                        self.mode == "cart" and now >= self.reverse_allowed_at and
                        now >= self.next_ramp_at):
                    steps = max(1, int((now - self.next_ramp_at) /
                                       self.ramp_interval) + 1)
                    for _ in range(steps):
                        self.current_left = move_toward(
                            self.current_left, self.target_left, self.ramp_step)
                        self.current_right = move_toward(
                            self.current_right, self.target_right, self.ramp_step)
                    self.next_ramp_at += steps * self.ramp_interval
                    self.current_command = self._format(
                        self.current_left, self.current_right)
                command = self.current_command
            if (command != previous or
                    time.monotonic() - self.last_write >= self.keepalive):
                self._write(command)
                return command
            return previous

    def close(self):
        self.stop_event.set()
        self.worker.join(timeout=1.0)
        self.emergency_stop("server shutdown")
        if self.serial is not None:
            self.serial.close()


@dataclass(order=True)
class DriveJob:
    execute_at_ns: int
    job_id: str = field(compare=False)
    command: str = field(compare=False)
    scheduled_clock_ns: int = field(compare=False)
    clock_mode: str = field(compare=False)
    owner: Optional["DriveSession"] = field(default=None, compare=False)
    committed: bool = field(default=False, compare=False)


class DriveScheduler:
    def __init__(self, driver):
        self.driver = driver
        self.cv = threading.Condition()
        self.jobs, self.heap = {}, []
        self.active_owner = None
        self.stopping = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def prepare(self, message, owner):
        command, _, _ = normalize_drive_command(message["command"])
        scheduled = int(message["execute_at_ns"])
        mode = message.get("clock_mode", "monotonic")
        now = time.monotonic_ns()
        deadline = (scheduled if mode == "monotonic" else
                    now + scheduled - clock_now_ns("tai"))
        if deadline <= now + 20_000_000:
            raise ValueError("execution deadline is less than 20 ms away")
        with self.cv:
            if message["job_id"] in self.jobs:
                raise ValueError("duplicate job id")
            job = DriveJob(deadline, message["job_id"], command, scheduled, mode, owner)
            self.jobs[job.job_id] = job
            heapq.heappush(self.heap, job)
            self.cv.notify_all()

    def commit(self, job_id, owner):
        with self.cv:
            job = self.jobs.get(job_id)
            if job is None or job.owner is not owner:
                raise ValueError("unknown job id")
            if (self.active_owner is not None and self.active_owner is not owner and
                    not self.active_owner.closed):
                raise ValueError("another master owns drive control")
            self.active_owner = owner
            job.committed = True
            owner.controls_motion = True
            self.cv.notify_all()

    def cancel(self, job_id):
        with self.cv:
            self.jobs.pop(job_id, None)
            self.cv.notify_all()

    def cancel_owner(self, owner):
        with self.cv:
            for job_id, job in list(self.jobs.items()):
                if job.owner is owner:
                    self.jobs.pop(job_id, None)
            if self.active_owner is owner:
                self.active_owner = None
            self.cv.notify_all()

    def _run(self):
        while True:
            with self.cv:
                if self.stopping:
                    return
                while self.heap and self.jobs.get(self.heap[0].job_id) is not self.heap[0]:
                    heapq.heappop(self.heap)
                if not self.heap:
                    self.cv.wait()
                    continue
                job = self.heap[0]
                now = time.monotonic_ns()
                if not job.committed:
                    if now >= job.execute_at_ns:
                        heapq.heappop(self.heap); self.jobs.pop(job.job_id, None)
                    else:
                        self.cv.wait((job.execute_at_ns - now) / 1e9)
                    continue
                remaining = job.execute_at_ns - now
                # Hand the job to apply_at early so it owns the UART lock before
                # the deadline; the ramp/keepalive thread cannot steal the slot.
                if remaining > 10_000_000:
                    self.cv.wait((remaining - 10_000_000) / 1e9)
                    continue
                heapq.heappop(self.heap); self.jobs.pop(job.job_id, None)
            try:
                actual = self.driver.apply_at(job.command, job.execute_at_ns,
                                              job.clock_mode)
                job.owner.send({"type": "executed", "job_id": job.job_id,
                                "scheduled_ns": job.scheduled_clock_ns,
                                "actual_ns": actual,
                                "late_ns": actual - job.scheduled_clock_ns})
            except Exception as exc:
                job.owner.send({"type": "execution_error", "job_id": job.job_id,
                                "error": str(exc)})

    def close(self):
        with self.cv:
            self.stopping = True; self.cv.notify_all()
        self.thread.join(timeout=1.0)


def send_message(stream, message):
    stream.write((json.dumps(message, separators=(",", ":")) + "\n").encode())
    stream.flush()


class DriveSession:
    def __init__(self, conn, address, scheduler, shutdown_event=None):
        self.conn, self.address, self.scheduler = conn, address, scheduler
        self.shutdown_event = shutdown_event
        self.stream = conn.makefile("rwb")
        self.send_lock = threading.Lock()
        self.last_seen = time.monotonic()
        self.controls_motion = False
        self.closed = False

    def send(self, message):
        with self.send_lock:
            send_message(self.stream, message)

    def run(self):
        print(f"Drive master connected: {self.address}", flush=True)
        try:
            for line in self.stream:
                received_mono = time.monotonic_ns()
                self.last_seen = time.monotonic()
                try:
                    message = json.loads(line)
                    kind = message.get("type")
                    if kind == "hello":
                        self.send({"type": "hello", "version": PROTOCOL_VERSION,
                                   "robot": socket.gethostname(),
                                   "dry_run": self.scheduler.driver.dry_run,
                                   "tai_available": tai_available()})
                    elif kind == "heartbeat":
                        pass
                    elif kind == "sync":
                        mode = message.get("clock_mode", "monotonic")
                        t1 = received_mono if mode == "monotonic" else clock_now_ns(mode)
                        self.send({"type": "sync", "seq": message["seq"],
                                   "t0_ns": message["t0_ns"], "t1_ns": t1,
                                   "t2_ns": clock_now_ns(mode), "clock_mode": mode})
                    elif kind == "prepare":
                        self.scheduler.prepare(message, self)
                        self.send({"type": "ready", "job_id": message["job_id"]})
                    elif kind == "commit":
                        self.scheduler.commit(message["job_id"], self)
                        self.send({"type": "committed", "job_id": message["job_id"]})
                    elif kind == "cancel":
                        self.scheduler.cancel(message["job_id"])
                    else:
                        raise ValueError(f"unknown message type: {kind}")
                except Exception as exc:
                    self.send({"type": "error", "error": str(exc)})
        finally:
            self.closed = True
            self.scheduler.cancel_owner(self)
            if self.controls_motion:
                self.scheduler.driver.emergency_stop("master disconnected")
            try:
                self.stream.close()
            finally:
                self.conn.close()
            print(f"Drive master disconnected: {self.address}", flush=True)
            if self.shutdown_event is not None:
                self.shutdown_event.set()


def run_slave(mode, default_port):
    parser = argparse.ArgumentParser(description=f"Synchronized {mode} drive slave")
    parser.add_argument("--listen", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=default_port)
    parser.add_argument("--serial-port", default="/dev/ttyS0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--serial-timeout", type=float, default=1.0)
    parser.add_argument("--keepalive", type=float, default=0.5)
    parser.add_argument("--ramp-step", type=float, default=0.005)
    parser.add_argument("--reverse-deadtime", type=float, default=0.35)
    parser.add_argument("--heartbeat-timeout", type=float, default=0.8)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--keep-alive", action="store_true",
                        help="keep listening after the master disconnects")
    args = parser.parse_args()
    driver = DriveSerial(mode, args.serial_port, args.baudrate, args.serial_timeout,
                         args.dry_run, args.keepalive, args.ramp_step,
                         args.reverse_deadtime)
    scheduler = DriveScheduler(driver)
    sessions, sessions_lock = [], threading.Lock()

    def watchdog():
        while not driver.stop_event.wait(0.1):
            with sessions_lock:
                active = list(sessions)
            for session in active:
                if (session.controls_motion and not session.closed and
                        time.monotonic() - session.last_seen > args.heartbeat_timeout):
                    scheduler.cancel_owner(session)
                    driver.emergency_stop("TCP heartbeat timeout")
                    session.controls_motion = False

    threading.Thread(target=watchdog, daemon=True).start()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.listen, args.port)); server.listen()
    server.settimeout(0.5)
    shutdown_event = threading.Event()
    print(f"{mode} drive slave listening on {args.listen}:{args.port}", flush=True)
    try:
        while not shutdown_event.is_set():
            try:
                conn, address = server.accept()
            except socket.timeout:
                continue
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            session = DriveSession(
                conn, address, scheduler,
                None if args.keep_alive else shutdown_event,
            )
            with sessions_lock:
                sessions.append(session)
            threading.Thread(target=session.run, daemon=True).start()
    except KeyboardInterrupt:
        pass
    finally:
        server.close(); scheduler.close(); driver.close()
