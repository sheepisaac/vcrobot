#!/usr/bin/env python3
"""Robot-side synchronized RoArm command server."""

import argparse
import heapq
import json
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Optional


TORQUE_ON_COMMAND = '{"T":210,"cmd":1}\n'
DEFA_OFF_COMMAND = '{"T":112,"mode":0,"b":1000,"s":1000,"e":1000,"h":1000}\n'
FEEDBACK_COMMAND = '{"T":105}\n'
DEFAULT_TORQUE_KEEPALIVE_INTERVAL = 0.2
DEFAULT_TORQUE_QUIET_AFTER_COMMAND = 1.5
DEFAULT_DEFA_RESTORE_INTERVAL = 0.0
PROTOCOL_VERSION = 2
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


def send_message(stream, message):
    stream.write((json.dumps(message, separators=(",", ":")) + "\n").encode())
    stream.flush()


class ArmSerial:
    def __init__(self, port, baudrate, timeout, dry_run=False,
                 torque_quiet_after_command=DEFAULT_TORQUE_QUIET_AFTER_COMMAND,
                 defa_restore_interval=DEFAULT_DEFA_RESTORE_INTERVAL,
                 torque_keepalive_interval=DEFAULT_TORQUE_KEEPALIVE_INTERVAL):
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.dry_run = dry_run
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.torque_quiet_after_command = max(0.0, float(torque_quiet_after_command))
        self.defa_restore_interval = max(0.0, float(defa_restore_interval))
        self.torque_keepalive_interval = max(0.05, float(torque_keepalive_interval))
        self.quiet_until_monotonic = 0.0
        self.last_defa_restore_monotonic = 0.0
        self.serial_module = None
        if dry_run:
            self.serial = None
            print("DRY-RUN: serial output is disabled", flush=True)
        else:
            import serial
            self.serial_module = serial
            self.serial = self._open_initialized_serial()
            print(f"Arm serial opened: {port} @ {baudrate}", flush=True)
            print(
                "Torque watchdog enabled: "
                f"T=210 every {self.torque_keepalive_interval:.3f}s, "
                f"T=112 restore every {self.defa_restore_interval:.3f}s, "
                f"quiet_after_command={self.torque_quiet_after_command:.3f}s",
                flush=True,
            )
        self.watchdog = threading.Thread(target=self._watchdog, daemon=True)
        self.watchdog.start()

    def _mark_controller_command(self, command):
        try:
            value = json.loads(command)
        except json.JSONDecodeError:
            return
        if isinstance(value, dict) and value.get("T") not in (105, 1051, 210):
            self.quiet_until_monotonic = (
                time.monotonic() + self.torque_quiet_after_command
            )

    def _open_initialized_serial(self):
        ser = self.serial_module.Serial(
            self.port, baudrate=self.baudrate, timeout=self.timeout, exclusive=True
        )
        ser.write(DEFA_OFF_COMMAND.encode())
        ser.write(TORQUE_ON_COMMAND.encode())
        ser.flush()
        return ser

    def write(self, command):
        data = command.rstrip("\r\n").encode() + b"\n"
        with self.lock:
            dispatched_ns = time.monotonic_ns()
            if self.serial is None and not self.dry_run:
                raise OSError("arm serial is disconnected")
            if self.serial is not None:
                self.serial.write(data)
                self.serial.flush()
            self._mark_controller_command(command)
        return dispatched_ns

    def _write_watchdog_command(self, command):
        data = command.rstrip("\r\n").encode() + b"\n"
        with self.lock:
            # A motion command may have extended the quiet window while the
            # watchdog was waiting for this lock. Recheck before touching UART.
            if (self.stop_event.is_set() or
                    time.monotonic() < self.quiet_until_monotonic):
                return False
            if self.serial is None and not self.dry_run:
                raise OSError("arm serial is disconnected")
            if self.serial is not None:
                self.serial.write(data)
                self.serial.flush()
            return True

    def write_at(self, command, execute_at_ns):
        """Reserve the serial writer just before a deadline, then dispatch on it."""
        data = command.rstrip("\r\n").encode() + b"\n"
        with self.lock:
            while True:
                remaining_ns = execute_at_ns - time.monotonic_ns()
                if remaining_ns <= 0:
                    break
                if remaining_ns > 1_000_000:
                    time.sleep((remaining_ns - 500_000) / 1e9)
                else:
                    # The final sub-millisecond spin avoids a full OS timer tick.
                    pass
            dispatched_ns = time.monotonic_ns()
            if self.serial is None and not self.dry_run:
                raise OSError("arm serial is disconnected")
            if self.serial is not None:
                self.serial.write(data)
                self.serial.flush()
            self._mark_controller_command(command)
        return dispatched_ns

    @staticmethod
    def _position_from_feedback(feedback):
        """Return all Cartesian fields exposed by this feedback sample."""
        containers = [feedback]
        for key in ("pos", "position", "cartesian", "pose"):
            if isinstance(feedback.get(key), dict):
                containers.append(feedback[key])
        for value in containers:
            position = {key: float(value[key]) for key in ("x", "y", "z", "t")
                        if isinstance(value.get(key), (int, float))}
            if position:
                return position
        return {}

    def _read_json_until(self, deadline):
        while time.monotonic() < deadline:
            raw = self.serial.readline()
            if not raw:
                continue
            try:
                value = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if isinstance(value, dict):
                return value
        return None

    def write_at_verified(self, command, execute_at_ns, clock_mode, verification):
        """Dispatch at a monotonic deadline and verify controller/pose feedback."""
        if self.dry_run:
            actual_mono_ns = self.write_at(command, execute_at_ns)
            return actual_mono_ns, clock_now_ns(clock_mode), {
                "success": False, "reason": "dry_run", "serial_ack": False,
                "command_ack": None, "position_reached": False,
            }
        if self.serial is None:
            raise OSError("arm serial is disconnected")

        data = command.rstrip("\r\n").encode() + b"\n"
        with self.lock:
            old_timeout = self.serial.timeout
            self.serial.timeout = 0.03
            try:
                self.serial.reset_input_buffer()
                while True:
                    remaining_ns = execute_at_ns - time.monotonic_ns()
                    if remaining_ns <= 0:
                        break
                    if remaining_ns > 1_000_000:
                        time.sleep((remaining_ns - 500_000) / 1e9)
                actual_mono_ns = time.monotonic_ns()
                actual_clock_ns = clock_now_ns(clock_mode)
                self.serial.write(data)
                self.serial.flush()
                self._mark_controller_command(command)

                ack_timeout = float(verification.get("ack_timeout", 0.3))
                direct_response = self._read_json_until(time.monotonic() + ack_timeout)
                command_ack = (direct_response if direct_response and
                               direct_response.get("T") in (104, 1041) else None)
                target = verification.get("target")
                if not isinstance(target, dict):
                    return actual_mono_ns, actual_clock_ns, {
                        "success": command_ack is not None,
                        "reason": "ack_received" if command_ack else "no_command_ack",
                        "serial_ack": direct_response is not None,
                        "command_ack": command_ack,
                        "direct_response": direct_response,
                        "position_reached": None,
                    }

                timeout = float(verification.get("timeout", 8.0))
                xyz_tolerance = float(verification.get("xyz_tolerance", 8.0))
                t_tolerance = float(verification.get("t_tolerance", 0.15))
                stable_required = max(1, int(verification.get("stable_samples", 2)))
                require_command_ack = bool(verification.get("require_command_ack", False))
                feedback_start_delay = max(
                    0.0, float(verification.get("feedback_start_delay", 0.0))
                )
                if feedback_start_delay:
                    time.sleep(feedback_start_delay)
                deadline = time.monotonic() + timeout
                stable = 0
                last_feedback = None
                position_state = {}
                last_errors = None
                feedback_seen = False
                complete_position_seen = False
                feedback_keys = set()

                while time.monotonic() < deadline:
                    self.serial.write(FEEDBACK_COMMAND.encode())
                    self.serial.flush()
                    feedback = self._read_json_until(min(deadline, time.monotonic() + 0.15))
                    if feedback is None or feedback.get("T") != 1051:
                        continue
                    feedback_seen = True
                    last_feedback = feedback
                    feedback_keys.update(feedback.keys())
                    position_state.update(self._position_from_feedback(feedback))
                    if not all(key in position_state for key in ("x", "y", "z", "t")):
                        time.sleep(0.05)
                        continue
                    complete_position_seen = True
                    position = dict(position_state)
                    errors = {key: abs(position[key] - float(target[key]))
                              for key in ("x", "y", "z")}
                    angle_delta = position["t"] - float(target["t"])
                    errors["t"] = abs((angle_delta + 3.141592653589793) %
                                      (2 * 3.141592653589793) - 3.141592653589793)
                    last_errors = errors
                    reached = (all(errors[key] <= xyz_tolerance
                                   for key in ("x", "y", "z")) and
                               errors["t"] <= t_tolerance)
                    stable = stable + 1 if reached else 0
                    if stable >= stable_required:
                        command_ok = command_ack is not None or not require_command_ack
                        return actual_mono_ns, actual_clock_ns, {
                            "success": command_ok,
                            "reason": ("position_reached" if command_ok
                                       else "position_reached_without_command_ack"),
                            "serial_ack": True,
                            "command_ack": command_ack,
                            "direct_response": direct_response,
                            "position_reached": True,
                            "position": position,
                            "errors": errors,
                            "stable_samples": stable,
                            "reached_at_ns": clock_now_ns(clock_mode),
                            "verification_duration_ms":
                                (time.monotonic_ns() - actual_mono_ns) / 1e6,
                        }
                    time.sleep(0.05)

                return actual_mono_ns, actual_clock_ns, {
                    "success": False,
                    "reason": ("position_timeout" if complete_position_seen else
                               "feedback_missing_cartesian_fields" if feedback_seen else
                               "no_controller_feedback"),
                    "serial_ack": feedback_seen or direct_response is not None,
                    "command_ack": command_ack,
                    "direct_response": direct_response,
                    "position_reached": False,
                    "position": position_state or None,
                    "errors": last_errors,
                    "feedback": last_feedback,
                    "feedback_keys": sorted(feedback_keys),
                    "missing_position_keys": sorted(
                        set(("x", "y", "z", "t")) - set(position_state)
                    ),
                }
            finally:
                self.serial.timeout = old_timeout

    def _watchdog(self):
        while not self.stop_event.wait(self.torque_keepalive_interval):
            if self.dry_run:
                continue
            try:
                if self.serial is None:
                    with self.lock:
                        if self.serial is None:
                            self.serial = self._open_initialized_serial()
                    print("Arm serial reconnected; DEFA disabled and torque restored",
                          flush=True)
                else:
                    if time.monotonic() < self.quiet_until_monotonic:
                        continue
                    # This fallback cannot prevent torque-OFF commands from
                    # another controller. Ordinary drive stops must avoid T=0.
                    now = time.monotonic()
                    if (self.defa_restore_interval > 0.0 and
                            now - self.last_defa_restore_monotonic >=
                            self.defa_restore_interval):
                        if self._write_watchdog_command(DEFA_OFF_COMMAND):
                            self.last_defa_restore_monotonic = now
                    self._write_watchdog_command(TORQUE_ON_COMMAND)
            except Exception as exc:
                print(f"Torque watchdog reconnecting after error: {exc}", flush=True)
                with self.lock:
                    if self.serial is not None:
                        try:
                            self.serial.close()
                        except Exception:
                            pass
                        self.serial = None

    def close(self):
        self.stop_event.set()
        self.watchdog.join(timeout=1.0)
        with self.lock:
            if self.serial is not None:
                self.serial.close()
                self.serial = None


@dataclass(order=True)
class Job:
    execute_at_ns: int
    job_id: str = field(compare=False)
    command: str = field(compare=False)
    clock_mode: str = field(default="monotonic", compare=False)
    scheduled_clock_ns: int = field(default=0, compare=False)
    verification: Optional[dict] = field(default=None, compare=False)
    committed: bool = field(default=False, compare=False)
    owner: Optional["ClientSession"] = field(default=None, compare=False)


class Scheduler:
    def __init__(self, arm):
        self.arm = arm
        self.cv = threading.Condition()
        self.jobs = {}
        self.heap = []
        self.stopping = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def prepare(self, job_id, command, execute_at_ns, owner,
                clock_mode="monotonic", verification=None):
        json.loads(command)  # Reject malformed controller commands early.
        scheduled_clock_ns = int(execute_at_ns)
        now = time.monotonic_ns()
        if clock_mode == "tai":
            execute_at_ns = now + (scheduled_clock_ns - clock_now_ns("tai"))
        elif clock_mode != "monotonic":
            raise ValueError(f"unsupported execution clock: {clock_mode}")
        if execute_at_ns <= now + 20_000_000:
            raise ValueError("execution deadline is less than 20 ms away")
        with self.cv:
            if job_id in self.jobs:
                raise ValueError("duplicate job id")
            job = Job(execute_at_ns, job_id, command, clock_mode,
                      scheduled_clock_ns, verification, owner=owner)
            self.jobs[job_id] = job
            heapq.heappush(self.heap, job)
            self.cv.notify_all()

    def commit(self, job_id):
        with self.cv:
            job = self.jobs.get(job_id)
            if job is None:
                raise ValueError("unknown job id")
            job.committed = True
            self.cv.notify_all()

    def cancel(self, job_id):
        with self.cv:
            self.jobs.pop(job_id, None)
            self.cv.notify_all()

    def cancel_owner(self, owner):
        with self.cv:
            for job_id, job in list(self.jobs.items()):
                if job.owner is owner and not job.committed:
                    self.jobs.pop(job_id, None)
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
                        heapq.heappop(self.heap)
                        self.jobs.pop(job.job_id, None)
                        continue
                    self.cv.wait((job.execute_at_ns - now) / 1e9)
                    continue
                remaining_ns = job.execute_at_ns - now
                if remaining_ns > 3_000_000:
                    self.cv.wait((remaining_ns - 3_000_000) / 1e9)
                    continue
                heapq.heappop(self.heap)
                self.jobs.pop(job.job_id, None)
            try:
                if job.verification:
                    actual_mono_ns, actual_ns, verification = self.arm.write_at_verified(
                        job.command, job.execute_at_ns, job.clock_mode, job.verification
                    )
                else:
                    actual_mono_ns = self.arm.write_at(job.command, job.execute_at_ns)
                    actual_ns = (actual_mono_ns if job.clock_mode == "monotonic"
                                 else clock_now_ns("tai"))
                    verification = None
                job.owner.send({
                    "type": "executed", "job_id": job.job_id,
                    "scheduled_ns": job.scheduled_clock_ns, "actual_ns": actual_ns,
                    "late_ns": actual_ns - job.scheduled_clock_ns,
                    "verification": verification,
                })
            except Exception as exc:
                job.owner.send({"type": "execution_error", "job_id": job.job_id,
                                "error": str(exc)})

    def close(self):
        with self.cv:
            self.stopping = True
            self.cv.notify_all()
        self.thread.join(timeout=1.0)


class ClientSession:
    def __init__(self, conn, address, scheduler, shutdown_event=None):
        self.conn = conn
        self.address = address
        self.scheduler = scheduler
        self.shutdown_event = shutdown_event
        self.stream = conn.makefile("rwb")
        self.send_lock = threading.Lock()

    def send(self, message):
        with self.send_lock:
            send_message(self.stream, message)

    def run(self):
        print(f"Master connected: {self.address}", flush=True)
        try:
            for raw_line in self.stream:
                received_ns = time.monotonic_ns()
                try:
                    message = json.loads(raw_line)
                    kind = message.get("type")
                    if kind == "hello":
                        self.send({"type": "hello", "version": PROTOCOL_VERSION,
                                   "robot": socket.gethostname(),
                                   "dry_run": self.scheduler.arm.dry_run,
                                   "tai_available": tai_available()})
                    elif kind == "sync":
                        clock_mode = message.get("clock_mode", "monotonic")
                        t1_ns = (received_ns if clock_mode == "monotonic"
                                 else clock_now_ns(clock_mode))
                        self.send({"type": "sync", "seq": message["seq"],
                                   "t0_ns": message["t0_ns"],
                                   "t1_ns": t1_ns,
                                   "t2_ns": clock_now_ns(clock_mode),
                                   "clock_mode": clock_mode})
                    elif kind == "prepare":
                        self.scheduler.prepare(message["job_id"], message["command"],
                                               int(message["execute_at_ns"]), self,
                                               message.get("clock_mode", "monotonic"),
                                               message.get("verification"))
                        self.send({"type": "ready", "job_id": message["job_id"]})
                    elif kind == "commit":
                        self.scheduler.commit(message["job_id"])
                        self.send({"type": "committed", "job_id": message["job_id"]})
                    elif kind == "cancel":
                        self.scheduler.cancel(message["job_id"])
                    else:
                        raise ValueError(f"unknown message type: {kind}")
                except Exception as exc:
                    self.send({"type": "error", "error": str(exc)})
        finally:
            self.scheduler.cancel_owner(self)
            try:
                self.stream.close()
            finally:
                self.conn.close()
            print(f"Master disconnected: {self.address}", flush=True)
            if self.shutdown_event is not None:
                self.shutdown_event.set()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=50210)
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--serial-timeout", type=float, default=2.0)
    parser.add_argument("--dry-run", action="store_true",
                        help="run networking and timing without opening the arm serial port")
    parser.add_argument("--torque-quiet-after-command", type=float,
                        default=DEFAULT_TORQUE_QUIET_AFTER_COMMAND,
                        help=("seconds to suppress torque keepalive after arm commands; "
                              "prevents T=210 from disturbing in-flight motion"))
    parser.add_argument("--defa-restore-interval", type=float,
                        default=DEFAULT_DEFA_RESTORE_INTERVAL,
                        help=("seconds between repeated T=112 DEFA/low-torque-mode "
                              "restore commands; 0 disables repeated restore"))
    parser.add_argument("--torque-keepalive-interval", type=float,
                        default=DEFAULT_TORQUE_KEEPALIVE_INTERVAL,
                        help="seconds between idle T=210 torque keepalive commands")
    parser.add_argument("--keep-alive", action="store_true",
                        help="keep listening after the master disconnects")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.torque_quiet_after_command < 0:
        raise SystemExit("--torque-quiet-after-command must be nonnegative")
    if args.defa_restore_interval < 0:
        raise SystemExit("--defa-restore-interval must be nonnegative")
    if args.torque_keepalive_interval <= 0:
        raise SystemExit("--torque-keepalive-interval must be positive")
    arm = ArmSerial(args.serial_port, args.baudrate, args.serial_timeout,
                    args.dry_run, args.torque_quiet_after_command,
                    args.defa_restore_interval,
                    args.torque_keepalive_interval)
    scheduler = Scheduler(arm)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.listen, args.port))
    server.listen()
    server.settimeout(0.5)
    shutdown_event = threading.Event()
    print(f"slave_arm listening on {args.listen}:{args.port}", flush=True)
    try:
        while not shutdown_event.is_set():
            try:
                conn, address = server.accept()
            except socket.timeout:
                continue
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            session = ClientSession(
                conn, address, scheduler,
                None if args.keep_alive else shutdown_event,
            )
            threading.Thread(target=session.run, daemon=True).start()
    except KeyboardInterrupt:
        print("Stopping slave_arm", flush=True)
    finally:
        server.close()
        scheduler.close()
        arm.close()


if __name__ == "__main__":
    main()
