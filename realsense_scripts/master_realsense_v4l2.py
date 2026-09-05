#!/usr/bin/env python3
"""Server-PC master for synchronized V4L2/ffmpeg YUV capture."""

import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
CTRL_DIR = SCRIPT_DIR.parent / "ctrl_scripts"
if str(CTRL_DIR) not in sys.path:
    sys.path.insert(0, str(CTRL_DIR))

from master_arm import Master, discover_slaves  # noqa: E402


DEFAULT_PORT = 50322

def parse_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "y", "on")


def parse_list(value):
    if value is None:
        return None
    return [item for item in re_split_commas_spaces(value) if item]


def re_split_commas_spaces(value):
    items = []
    for chunk in str(value).replace(",", " ").split():
        stripped = chunk.strip()
        if stripped:
            items.append(stripped)
    return items


def parse_parameter_file(path):
    parameters = {}
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path.resolve()}")
    current_section = None
    for raw_line in path.read_text(encoding="utf-8-sig").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or line.startswith(";"):
            continue
        if line.startswith("[") and line.endswith("]"):
            current_section = line[1:-1].strip().lower()
            parameters.setdefault(current_section, {})
            continue
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if "#" in value:
            value = value.split("#", 1)[0].strip()
        if ";" in value:
            value = value.split(";", 1)[0].strip()
        section = current_section or "default"
        parameters.setdefault(section, {})[key] = value
    return parameters


def auto_capture_frames(target_frames):
    return target_frames


def endpoints(args):
    if args.robots:
        return [value if ":" in value else f"{value}:{args.port}" for value in args.robots]
    found = discover_slaves(
        args.discover_subnet, args.port, args.discovery_timeout,
        args.discovery_workers, expected=args.expected
    )
    if len(found) != args.expected:
        raise SystemExit(f"Discovery found {len(found)} slave(s), expected {args.expected}")
    return [f"{host}:{args.port}" for host in found]


def parse_args():
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--params", default=str(SCRIPT_DIR / "input_parameters.txt"))
    pre_args, _ = pre_parser.parse_known_args()
    params = parse_parameter_file(pre_args.params)
    master = params.get("master", {})
    slave = params.get("slave", {})

    parser = argparse.ArgumentParser(description=__doc__, parents=[pre_parser])
    parser.add_argument("--robots", nargs="+", default=parse_list(master.get("robots")))
    parser.add_argument("--port", type=int, default=int(master.get("port", DEFAULT_PORT)))
    parser.add_argument("--discover-subnet", default=master.get("discover_subnet", "192.168.10.0/24"))
    parser.add_argument("--discovery-timeout", type=float, default=float(master.get("discovery_timeout", 0.75)))
    parser.add_argument("--discovery-workers", type=int, default=int(master.get("discovery_workers", 32)))
    parser.add_argument("--expected", type=int, default=int(master.get("expected", 2)))
    parser.add_argument("--target-frames", type=int,
                        default=int(master["target_frames"]) if "target_frames" in master else None)
    parser.add_argument("--width", type=int, default=int(master.get("width", slave.get("width", 1920))))
    parser.add_argument("--height", type=int, default=int(master.get("height", slave.get("height", 1080))))
    parser.add_argument("--fps", type=float, default=float(master.get("fps", slave.get("fps", 30.0))))
    parser.add_argument("--device",
                        default=master.get("device"),
                        help="override slave V4L2 device, e.g. /dev/video2")
    parser.add_argument("--input-format", default=master.get("input_format", slave.get("input_format", "yuyv422")),
                        help="V4L2 input format: yuyv422 or mjpeg are common")
    parser.add_argument("--output-pix-fmt", default=master.get("output_pix_fmt", slave.get("output_pix_fmt", "yuv420p")),
                        choices=("yuyv422", "yuv420p", "nv12", "rgb24", "bgr24"))
    parser.add_argument("--color-range", default=master.get("color_range", slave.get("color_range", "pc")),
                        help="ffmpeg color range, e.g. pc for full-range or tv for limited")
    parser.add_argument("--colorspace", default=master.get("colorspace", slave.get("colorspace", "bt709")))
    parser.add_argument("--color-primaries", default=master.get("color_primaries", slave.get("color_primaries", "bt709")))
    parser.add_argument("--color-trc", default=master.get("color_trc", slave.get("color_trc", "bt709")))
    parser.add_argument("--in-range", default=master.get("in_range", slave.get("in_range", "pc")))
    parser.add_argument("--in-color-matrix", default=master.get("in_color_matrix", slave.get("in_color_matrix", "bt709")))
    parser.add_argument("--lead", type=float, default=float(master.get("lead", 2.0)))
    parser.add_argument("--timeout", type=float, default=float(master.get("timeout", 90.0)))
    parser.add_argument("--capture-timeout", type=float, default=float(master.get("capture_timeout", 60.0)))
    parser.add_argument("--ready-timeout", type=float, default=float(master.get("ready_timeout", 60.0)),
                        help="wait for all cameras to finish warmup/stability checks before scheduling")
    parser.add_argument("--sync-samples", type=int, default=int(master.get("sync_samples", 25)))
    parser.add_argument("--clock-mode", choices=("estimated", "ptp"), default=master.get("clock_mode", "estimated"))
    parser.add_argument("--ptp-max-offset-ms", type=float, default=float(master.get("ptp_max_offset_ms", 1.0)))
    parser.add_argument("--output-dir", default=master.get("output_dir", slave.get("output_dir", "./Results")))
    args = parser.parse_args()
    if args.target_frames is None:
        parser.error("--target-frames is required, or set target_frames in [master] of input_parameters.txt")
    if args.target_frames < 1:
        parser.error("--target-frames must be positive")
    if args.lead < 0.2:
        parser.error("--lead must be at least 0.2")
    if args.sync_samples < 3:
        parser.error("--sync-samples must be at least 3")
    for name in ("ready_timeout", "capture_timeout", "timeout", "fps", "lead"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"{name} must be finite and positive")
    print(f"Parameters: {Path(args.params).resolve()}", flush=True)
    args.capture_frames = auto_capture_frames(args.target_frames)
    return args


def wait_for_cameras(master, timeout):
    """Readiness barrier precedes dispatch's clock sync and common start time."""
    deadline = time.monotonic() + timeout
    previous = {}
    print("Waiting for every camera to finish warmup and image stabilization...", flush=True)
    while True:
        if time.monotonic() >= deadline:
            raise RuntimeError(f"Camera readiness timed out after {timeout}s: {previous}")
        # Ask all slaves before collecting replies; readiness never opens a camera.
        saved_timeouts = [robot.socket.gettimeout() for robot in master.robots]
        try:
            for robot in master.robots:
                robot.socket.settimeout(max(.01, min(2.0, deadline - time.monotonic())))
                robot.send({"type": "camera_status"})
            statuses = []
            for robot in master.robots:
                robot.socket.settimeout(max(.01, min(2.0, deadline - time.monotonic())))
                reply, _ = robot.receive()
                if reply.get("type") != "camera_status":
                    raise RuntimeError(f"{robot.endpoint}: update slave_realsense_v4l2.py; "
                                       f"expected camera_status, got {reply}")
                if reply.get("stream_error"):
                    raise RuntimeError(f"{robot.endpoint}: {reply['stream_error']}")
                state = (reply.get("ready"), reply.get("reason"))
                if previous.get(robot.endpoint) != state:
                    print(f"  {robot.endpoint}: {state[1]}; "
                          f"frames_seen={reply.get('frames_seen')}; "
                          f"mean_y={reply.get('mean_y')}", flush=True)
                    previous[robot.endpoint] = state
                statuses.append(reply)
        finally:
            for robot, old_timeout in zip(master.robots, saved_timeouts):
                robot.socket.settimeout(old_timeout)
        if statuses and all(status.get("ready") is True for status in statuses):
            print("ALL CAMERAS READY. Scheduling capture from the running streams.", flush=True)
            return statuses
        time.sleep(min(.25, max(0.0, deadline - time.monotonic())))


def print_report(results):
    print("V4L2/ffmpeg execution report:")
    actual = [item["actual_master_ns"] for item in results]
    if actual:
        print(f"  capture command skew={(max(actual)-min(actual))/1e6:.3f} ms")
    start_lates = []
    for item in results:
        verification = item.get("verification") or {}
        capture = verification.get("capture") or {}
        if capture.get("start_late_ms") is not None:
            start_lates.append(float(capture["start_late_ms"]))
        print(
            f"  {item['robot'].endpoint}: success={verification.get('success')} "
            f"reason={verification.get('reason')} "
            f"file={capture.get('filename')} csv={capture.get('csv_filename')} "
            f"log={capture.get('log_filename')} "
            f"meta={capture.get('meta_filename')} "
            f"frames={capture.get('saved_frames', capture.get('showinfo_frames'))}/"
            f"{capture.get('target_frames', capture.get('frame_count'))} "
            f"size={capture.get('file_size_bytes')} "
            f"camera_ready={capture.get('camera_ready')} "
            f"startup_discarded={capture.get('startup_frames_discarded')} "
            f"start_late_ms={capture.get('start_late_ms')}"
        )
    if len(start_lates) > 1:
        print(f"  capture-call start-late skew={max(start_lates)-min(start_lates):.3f} ms")
    print("  note: ffmpeg camera streams are already open on slaves; CSV has per-frame read timestamps.")


def main():
    args = parse_args()
    session_dt = datetime.now()
    session_date = session_dt.strftime("%Y%m%d")
    session_time = session_dt.strftime("%H%M")
    session_stamp = f"{session_date}_{session_time}"
    print(
        f"Target frames={args.target_frames}; capture frames={args.capture_frames} "
        "(camera stream is pre-opened on each slave)"
    )
    print(f"Master session timestamp: {session_stamp}")
    command = {
        "action": "v4l2_video",
        "session_date": session_date,
        "session_time": session_time,
        "session_stamp": session_stamp,
        "target_frames": args.target_frames,
        "frame_count": args.capture_frames,
        "capture_frames": args.capture_frames,
        "width": args.width,
        "height": args.height,
        "fps": args.fps,
        "input_format": args.input_format,
        "output_pix_fmt": args.output_pix_fmt,
        "color_range": args.color_range,
        "colorspace": args.colorspace,
        "color_primaries": args.color_primaries,
        "color_trc": args.color_trc,
        "in_range": args.in_range,
        "in_color_matrix": args.in_color_matrix,
        "output_dir": args.output_dir,
        "capture_timeout": args.capture_timeout,
    }
    if args.device:
        command["device"] = args.device
    eps = endpoints(args)
    master = Master(
        eps, args.timeout, args.sync_samples,
        args.clock_mode, args.ptp_max_offset_ms
    )
    try:
        wait_for_cameras(master, args.ready_timeout)
        # Date/time belongs to this ready capture, not the start of warmup.
        session_dt = datetime.now()
        command.update(session_date=session_dt.strftime("%Y%m%d"),
                       session_time=session_dt.strftime("%H%M"),
                       session_stamp=session_dt.strftime("%Y%m%d_%H%M"))
        print(f"Capture session timestamp: {command['session_stamp']}", flush=True)
        results = master.dispatch(
            [(0.0, "v4l2_realsense", json.dumps(command, separators=(",", ":")))],
            args.lead, report=False
        )
        print_report(results)
        if any(not (item.get("verification") or {}).get("success") or
               (item.get("verification", {}).get("capture") or {}).get("saved_frames") != args.target_frames
               for item in results):
            raise RuntimeError("Capture failed: not every robot saved the requested frame count; see report")
    finally:
        master.close()


if __name__ == "__main__":
    main()
