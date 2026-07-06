#!/usr/bin/env python3
"""Server-PC master for synchronized RoArm commands."""

import argparse
import concurrent.futures
import ipaddress
import json
import socket
import statistics
import threading
import time
import uuid


PROTOCOL_VERSION = 2
DEFAULT_PORT = 50210
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
    raise RuntimeError(f"clock mode {mode!r} is unavailable on this host")


def parse_endpoint(value):
    if ":" in value:
        host, port = value.rsplit(":", 1)
        return host, int(port)
    return value, DEFAULT_PORT


def probe_slave(address, port, timeout):
    """Return slave identity only when a TCP endpoint speaks our protocol."""
    try:
        with socket.create_connection((str(address), port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            stream = sock.makefile("rwb")
            try:
                message = {"type": "hello", "version": PROTOCOL_VERSION}
                stream.write((json.dumps(message, separators=(",", ":")) + "\n").encode())
                stream.flush()
                line = stream.readline()
                if not line:
                    return None
                reply = json.loads(line)
                if (reply.get("type") == "hello" and
                        reply.get("version") == PROTOCOL_VERSION):
                    return str(address), reply.get("robot", str(address))
            finally:
                stream.close()
    except (OSError, ValueError, json.JSONDecodeError):
        return None
    return None


def discover_slaves(subnet, port, timeout, workers=32, attempts=2, expected=None):
    """Find compatible slave_arm servers using TCP only."""
    network = ipaddress.ip_network(subnet, strict=False)
    if network.version != 4:
        raise ValueError("TCP subnet discovery currently supports IPv4 only")
    addresses = list(network.hosts())
    print(f"Discovering arm slaves on {network} TCP/{port} ...", flush=True)
    found_by_address = {}
    remaining = addresses
    for attempt in range(1, attempts + 1):
        with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as executor:
            futures = [executor.submit(probe_slave, address, port, timeout)
                       for address in remaining]
            for future in concurrent.futures.as_completed(futures):
                result = future.result()
                if result is not None and result[0] not in found_by_address:
                    found_by_address[result[0]] = result
                    print(f"  found {result[1]} at {result[0]}:{port}", flush=True)
        if attempt == attempts or (expected is not None and
                                   len(found_by_address) >= expected):
            break
        remaining = [address for address in addresses
                     if str(address) not in found_by_address]
        print("  retrying unanswered addresses after ARP warm-up ...", flush=True)
        time.sleep(0.5)
    found = list(found_by_address.values())
    return [address for address, _ in sorted(found, key=lambda item: ipaddress.ip_address(item[0]))]


class Robot:
    def __init__(self, endpoint, timeout):
        self.endpoint = endpoint
        self.host, self.port = parse_endpoint(endpoint)
        self.socket = socket.create_connection((self.host, self.port), timeout=timeout)
        # A scheduled start/end may legitimately be farther away than connect timeout.
        self.socket.settimeout(None)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.stream = self.socket.makefile("rwb")
        self.send_lock = threading.Lock()
        self.offset_ns = 0
        self.delay_ns = 0
        self.send({"type": "hello", "version": PROTOCOL_VERSION})
        reply, _ = self.receive()
        if reply.get("type") != "hello" or reply.get("version") != PROTOCOL_VERSION:
            raise RuntimeError(f"{endpoint}: incompatible slave: {reply}")
        self.name = reply.get("robot", endpoint)
        self.dry_run = bool(reply.get("dry_run", False))
        self.tai_available = bool(reply.get("tai_available", False))

    def send(self, message):
        line = json.dumps(message, separators=(",", ":")) + "\n"
        with self.send_lock:
            self.stream.write(line.encode())
            self.stream.flush()

    def receive(self):
        line = self.stream.readline()
        received_ns = time.monotonic_ns()
        if not line:
            raise ConnectionError(f"{self.endpoint}: connection closed")
        message = json.loads(line)
        if message.get("type") == "error":
            raise RuntimeError(f"{self.endpoint}: {message.get('error')}")
        return message, received_ns

    def request(self, message, expected):
        self.send(message)
        reply, received_ns = self.receive()
        if reply.get("type") != expected:
            raise RuntimeError(f"{self.endpoint}: expected {expected}, got {reply}")
        return reply, received_ns

    def synchronize(self, samples, clock_mode="monotonic"):
        measurements = []
        for seq in range(samples):
            t0 = clock_now_ns(clock_mode)
            reply, _ = self.request(
                {"type": "sync", "seq": seq, "t0_ns": t0,
                 "clock_mode": clock_mode}, "sync"
            )
            t3 = clock_now_ns(clock_mode)
            t1, t2 = int(reply["t1_ns"]), int(reply["t2_ns"])
            delay = (t3 - t0) - (t2 - t1)
            offset = ((t1 - t0) + (t2 - t3)) // 2
            measurements.append((delay, offset))
            time.sleep(0.01)
        best = sorted(measurements)[:max(1, min(5, len(measurements)))]
        self.delay_ns = min(item[0] for item in best)
        self.offset_ns = int(statistics.median(item[1] for item in best))

    def close(self):
        try:
            self.stream.close()
        finally:
            self.socket.close()


class Master:
    def __init__(self, endpoints, timeout, samples, clock_mode="estimated",
                 ptp_max_offset_ms=1.0):
        self.robots = []
        self.sync_samples = samples
        self.clock_mode = clock_mode
        self.ptp_max_offset_ns = int(ptp_max_offset_ms * 1e6)
        sync_clock = "tai" if clock_mode == "ptp" else "monotonic"
        if clock_mode == "ptp" and not tai_available():
            raise RuntimeError("PTP mode requires CLOCK_TAI on the master host")
        try:
            for endpoint in endpoints:
                print(f"Connecting to arm slave at {endpoint} ...", flush=True)
                self.robots.append(Robot(endpoint, timeout))
            for robot in self.robots:
                if clock_mode == "ptp" and not robot.tai_available:
                    raise RuntimeError(f"{robot.endpoint}: CLOCK_TAI is unavailable")
                robot.synchronize(samples, sync_clock)
                mode = "DRY-RUN" if robot.dry_run else "SERIAL"
                print(f"Connected {robot.endpoint} ({robot.name}): "
                      f"mode={mode}, "
                      f"RTT floor {robot.delay_ns / 1e6:.3f} ms, "
                      f"clock offset {robot.offset_ns / 1e6:+.3f} ms")
                if (clock_mode == "ptp" and
                        abs(robot.offset_ns) > self.ptp_max_offset_ns):
                    raise RuntimeError(
                        f"{robot.endpoint}: PTP/TAI offset "
                        f"{robot.offset_ns / 1e6:+.3f} ms exceeds "
                        f"{ptp_max_offset_ms:.3f} ms"
                    )
            dry_robots = [robot.endpoint for robot in self.robots if robot.dry_run]
            if dry_robots:
                print("WARNING: no physical arm motion in DRY-RUN mode: " +
                      ", ".join(dry_robots), flush=True)
        except Exception:
            self.close()
            raise

    def dispatch(self, commands, lead_seconds, report=True):
        """Commands are (offset, label, JSON[, verification]) tuples."""
        # Refresh immediately before every action so oscillator drift cannot build up.
        sync_clock = "tai" if self.clock_mode == "ptp" else "monotonic"
        for robot in self.robots:
            robot.synchronize(self.sync_samples, sync_clock)
            if (self.clock_mode == "ptp" and
                    abs(robot.offset_ns) > self.ptp_max_offset_ns):
                raise RuntimeError(
                    f"{robot.endpoint}: PTP offset drifted to "
                    f"{robot.offset_ns / 1e6:+.3f} ms; command aborted"
                )
        base_ns = clock_now_ns(sync_clock) + int(lead_seconds * 1e9)
        jobs = []
        try:
            for robot in self.robots:
                for command_spec in commands:
                    offset_seconds, label, command = command_spec[:3]
                    verification = command_spec[3] if len(command_spec) > 3 else None
                    job_id = uuid.uuid4().hex
                    execute_master_ns = base_ns + int(offset_seconds * 1e9)
                    execute_slave_ns = (execute_master_ns if self.clock_mode == "ptp"
                                        else execute_master_ns + robot.offset_ns)
                    robot.send({"type": "prepare", "job_id": job_id,
                                "execute_at_ns": execute_slave_ns, "command": command,
                                "clock_mode": sync_clock,
                                "verification": verification})
                    jobs.append({"robot": robot, "id": job_id, "label": label,
                                 "master_ns": execute_master_ns,
                                 "slave_ns": execute_slave_ns, "ready": False})
            for job in jobs:
                reply, _ = job["robot"].receive()
                if reply.get("type") != "ready" or reply.get("job_id") != job["id"]:
                    raise RuntimeError(f"prepare failed: {reply}")
                job["ready"] = True
            if clock_now_ns(sync_clock) >= base_ns - 50_000_000:
                raise RuntimeError("not enough lead time after READY; increase --lead")
            for job in jobs:
                job["robot"].send({"type": "commit", "job_id": job["id"]})
            for job in jobs:
                reply, _ = job["robot"].receive()
                if reply.get("type") != "committed" or reply.get("job_id") != job["id"]:
                    raise RuntimeError(f"commit failed: {reply}")
            results = []
            pending = {(job["robot"].endpoint, job["id"]): job for job in jobs}
            # Each robot reports jobs in chronological order. Read in that order.
            for robot in self.robots:
                robot_jobs = sorted((j for j in jobs if j["robot"] is robot),
                                    key=lambda j: j["slave_ns"])
                for _ in robot_jobs:
                    reply, _ = robot.receive()
                    if reply.get("type") != "executed":
                        raise RuntimeError(f"execution failed: {reply}")
                    job = pending.pop((robot.endpoint, reply["job_id"]))
                    actual_master_ns = (int(reply["actual_ns"])
                                        if self.clock_mode == "ptp"
                                        else int(reply["actual_ns"]) - robot.offset_ns)
                    job["actual_master_ns"] = actual_master_ns
                    job["late_ns"] = actual_master_ns - job["master_ns"]
                    job["verification"] = reply.get("verification")
                    verification = job["verification"] or {}
                    if verification.get("reached_at_ns") is not None:
                        reached_ns = int(verification["reached_at_ns"])
                        job["reached_master_ns"] = (
                            reached_ns if self.clock_mode == "ptp"
                            else reached_ns - robot.offset_ns
                        )
                    results.append(job)
            if report:
                self._print_results(results)
            return results
        except Exception:
            for job in jobs:
                if job["ready"]:
                    try:
                        job["robot"].send({"type": "cancel", "job_id": job["id"]})
                    except Exception:
                        pass
            raise

    @staticmethod
    def _print_results(results):
        print("Execution report:")
        labels = sorted(set(item["label"] for item in results))
        for label in labels:
            group = [item for item in results if item["label"] == label]
            actual = [item["actual_master_ns"] for item in group]
            skew_ms = (max(actual) - min(actual)) / 1e6 if actual else 0.0
            detail = ", ".join(
                f"{item['robot'].endpoint} late={item['late_ns'] / 1e6:+.3f}ms"
                for item in group
            )
            print(f"  {label}: observed skew={skew_ms:.3f} ms; {detail}")
            for item in group:
                verification = item.get("verification")
                if verification is not None:
                    print(f"    {item['robot'].endpoint}: verify="
                          f"{verification.get('success')} "
                          f"reason={verification.get('reason')} "
                          f"serial_ack={verification.get('serial_ack')} "
                          f"command_ack={verification.get('command_ack')} "
                          f"position={verification.get('position')} "
                          f"errors={verification.get('errors')} "
                          f"feedback_keys={verification.get('feedback_keys')}")
                    if verification.get("reason") == "feedback_missing_cartesian_fields":
                        print(f"      raw_feedback={verification.get('feedback')}")
            reached = [item["reached_master_ns"] for item in group
                       if "reached_master_ns" in item]
            if len(reached) == len(group) and len(reached) > 1:
                print(f"    target completion skew="
                      f"{(max(reached) - min(reached)) / 1e6:.3f} ms")

    def close(self):
        for robot in self.robots:
            try:
                robot.close()
            except Exception:
                pass
        self.robots = []


def validate_command(command):
    value = json.loads(command)
    if not isinstance(value, dict):
        raise ValueError("arm command must be a JSON object")
    return json.dumps(value, separators=(",", ":"))


def interactive(master, lead):
    print("Enter an arm JSON command, or:")
    print("  run SECONDS START_JSON || END_JSON")
    print("  sync   (remeasure clocks)")
    print("  quit")
    while True:
        try:
            text = input("arm-master> ").strip()
            if not text:
                continue
            if text.lower() in ("quit", "exit"):
                return
            if text.lower() == "sync":
                for robot in master.robots:
                    robot.synchronize(15)
                    print(f"{robot.endpoint}: RTT={robot.delay_ns / 1e6:.3f}ms "
                          f"offset={robot.offset_ns / 1e6:+.3f}ms")
                continue
            if text.startswith("run "):
                header, separator, end = text.partition("||")
                if not separator:
                    raise ValueError("run syntax requires '|| END_JSON'")
                _, duration, start = header.split(maxsplit=2)
                duration = float(duration)
                if duration <= 0:
                    raise ValueError("duration must be positive")
                commands = [(0.0, "start", validate_command(start)),
                            (duration, "end", validate_command(end.strip()))]
            else:
                commands = [(0.0, "command", validate_command(text))]
            master.dispatch(commands, lead)
        except (ValueError, RuntimeError, OSError, json.JSONDecodeError) as exc:
            print(f"Error: {exc}")
        except KeyboardInterrupt:
            print()
            return


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", nargs="+",
                        help="explicit robot hostnames/IPs; disables discovery")
    parser.add_argument("--discover-subnet", default="192.168.10.0/24",
                        help="IPv4 subnet scanned for slave_arm TCP servers")
    parser.add_argument("--discovery-port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--discovery-timeout", type=float, default=0.75,
                        help="per-address TCP discovery timeout in seconds")
    parser.add_argument("--discovery-workers", type=int, default=32,
                        help="maximum simultaneous TCP discovery attempts")
    parser.add_argument("--expected", type=int, default=2,
                        help="required number of discovered slaves")
    parser.add_argument("--lead", type=float, default=1.0,
                        help="seconds between READY phase and execution")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--sync-samples", type=int, default=15)
    parser.add_argument("--command", help="send one JSON command and exit")
    parser.add_argument("--end-command", help="JSON command sent after --duration")
    parser.add_argument("--duration", type=float,
                        help="seconds from --command to --end-command")
    args = parser.parse_args()
    if args.lead < 0.2:
        parser.error("--lead must be at least 0.2 seconds")
    if args.sync_samples < 3:
        parser.error("--sync-samples must be at least 3")
    if args.expected < 1:
        parser.error("--expected must be at least 1")
    if args.discovery_timeout <= 0:
        parser.error("--discovery-timeout must be positive")
    if args.discovery_workers < 1:
        parser.error("--discovery-workers must be at least 1")
    if bool(args.end_command) != (args.duration is not None):
        parser.error("--end-command and --duration must be used together")
    return args


def main():
    args = parse_args()
    if args.robots:
        endpoints = args.robots
    else:
        try:
            endpoints = discover_slaves(
                args.discover_subnet, args.discovery_port, args.discovery_timeout,
                args.discovery_workers, expected=args.expected
            )
        except ValueError as exc:
            raise SystemExit(f"Discovery configuration error: {exc}")
        if len(endpoints) != args.expected:
            raise SystemExit(
                f"Discovery found {len(endpoints)} slave(s), expected {args.expected}.\n"
                "Start slave_arm.py on every robot, check the subnet/firewall, or "
                "use --robots IP [IP ...]."
            )
    try:
        master = Master(endpoints, args.timeout, args.sync_samples)
    except (OSError, RuntimeError, ConnectionError) as exc:
        targets = ", ".join(endpoints)
        raise SystemExit(
            f"Could not connect to all arm slaves ({targets}): {exc}\n"
            "Start slave_arm.py on every robot and allow TCP port 50210."
        )
    try:
        if args.command:
            commands = [(0.0, "start" if args.end_command else "command",
                         validate_command(args.command))]
            if args.end_command:
                if args.duration <= 0:
                    raise ValueError("--duration must be positive")
                commands.append((args.duration, "end",
                                 validate_command(args.end_command)))
            master.dispatch(commands, args.lead)
        else:
            interactive(master, args.lead)
    finally:
        master.close()


if __name__ == "__main__":
    main()
