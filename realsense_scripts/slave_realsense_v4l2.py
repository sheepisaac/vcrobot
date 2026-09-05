#!/usr/bin/env python3
"""Robot-side V4L2/ffmpeg TCP slave for synchronized YUV capture.

This path bypasses ROS2 and pyrealsense2.  The slave waits for the same TCP
prepare/commit protocol used by the arm sync scripts. One persistent ffmpeg
stream warms up before the master can schedule saving frames.

Output is raw YUV plus a CSV sidecar.  The raw file intentionally has no
container timestamps; CSV timestamps measure pipe reads, not sensor exposure.
"""

import argparse
from collections import deque
import csv
import json
import math
import re
import shutil
import socket
import subprocess
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional


PROTOCOL_VERSION = 2
DEFAULT_PORT = 50322
TAI_CLOCK_ID = getattr(time, "CLOCK_TAI", 11)

SHOWINFO_RE = re.compile(
    r"n:\s*(?P<n>\d+).*?pts:\s*(?P<pts>-?\d+).*?pts_time:\s*(?P<pts_time>-?[0-9.]+)"
)

PIX_FMT_EXT = {
    "yuyv422": "yuyv",
    "yuv420p": "yuv",
    "nv12": "nv12",
    "rgb24": "rgb",
    "bgr24": "bgr",
}

PIX_FMT_LABEL = {
    "yuyv422": "yuyv422",
    "yuv420p": "yuv420",
    "nv12": "nv12",
    "rgb24": "rgb24",
    "bgr24": "bgr24",
}

SAFE_NAME_RE = re.compile(r"[^A-Za-z0-9_.-]+")


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


def require_executable(name):
    path = shutil.which(name)
    if path is None:
        raise RuntimeError(f"required executable not found: {name}")
    return path


def parse_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "y", "on")


def parse_parameter_file(path):
    """Parse a small INI-like key=value file without external dependencies."""
    parameters = {}
    current_section = None
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path.resolve()}")
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


def section_value(parameters, section, key, default=None):
    return parameters.get(section, {}).get(key, parameters.get("default", {}).get(key, default))


def camera_controls(parameters):
    controls = []
    for key, value in parameters.get("camera_controls", {}).items():
        controls.append((key, value))
    for key, value in parameters.get("controls", {}).items():
        controls.append((key, value))
    return controls


def safe_name(value, fallback):
    text = str(value or fallback).strip()
    text = SAFE_NAME_RE.sub("_", text)
    return text or fallback


class FrameReadiness:
    """Sampled image stability is a startup heuristic, not color calibration."""

    def __init__(self, min_frames=90, warmup_seconds=5.0, stable_seconds=2.0,
                 luma_tolerance=3.0, chroma_tolerance=2.0, min_luma=25.0):
        values = (warmup_seconds, stable_seconds, luma_tolerance,
                  chroma_tolerance, min_luma)
        if min_frames < 1 or any(not math.isfinite(v) or v < 0 for v in values):
            raise ValueError("Readiness parameters must be finite and nonnegative")
        if stable_seconds <= 0:
            raise ValueError("stable_seconds must be positive")
        self.min_frames = min_frames
        self.warmup_seconds = warmup_seconds
        self.stable_seconds = stable_seconds
        self.luma_tolerance = luma_tolerance
        self.chroma_tolerance = chroma_tolerance
        self.min_luma = min_luma
        self.first_ns = None
        self.last_ns = None
        self.count = 0
        self.history = deque()
        self.ready_ns = None
        self.ready_frame_idx = None
        self.ready_metrics = None
        self.metrics = {}
        self.reason = "waiting_for_frames"

    @staticmethod
    def measure(data, width, height, pix_fmt):
        view = memoryview(data)
        pixels = width * height

        def mean(start, count, stride=1):
            # About 1024 samples/plane; never convert or copy a full frame.
            step = max(1, count // 1024)
            samples = view[start:start + count * stride:stride * step]
            return sum(samples) / len(samples)

        if pix_fmt == "yuv420p":
            return (mean(0, pixels), mean(pixels, pixels // 4),
                    mean(pixels * 5 // 4, pixels // 4))
        if pix_fmt == "nv12":
            return (mean(0, pixels), mean(pixels, pixels // 4, 2),
                    mean(pixels + 1, pixels // 4, 2))
        if pix_fmt == "yuyv422":
            return (mean(0, pixels, 2), mean(1, pixels // 2, 4),
                    mean(3, pixels // 2, 4))
        channels = [mean(i, pixels, 3) for i in range(3)]
        r, g, b = channels if pix_fmt == "rgb24" else channels[::-1]
        y = .2126 * r + .7152 * g + .0722 * b
        return y, 128 + (b - y) / 1.8556, 128 + (r - y) / 1.5748

    def observe(self, frame_idx, now_ns, metrics):
        if self.first_ns is None:
            self.first_ns = now_ns
        if self.last_ns is not None and now_ns - self.last_ns > 1_000_000_000:
            self.first_ns = now_ns
            self.count = 0
            self.history.clear()
            self.ready_ns = self.ready_frame_idx = None
            self.ready_metrics = None
        self.last_ns = now_ns
        self.count += 1
        self.metrics = dict(zip(("mean_y", "mean_u", "mean_v"), metrics))
        if self.ready_ns is not None:
            return False
        elapsed = (now_ns - self.first_ns) / 1e9
        if self.count < self.min_frames or elapsed < self.warmup_seconds:
            self.reason = "warming_up"
            return False
        self.history.append((now_ns, metrics))
        cutoff = now_ns - int(self.stable_seconds * 1e9)
        while len(self.history) > 1 and self.history[1][0] <= cutoff:
            self.history.popleft()
        if self.history[0][0] > cutoff or len(self.history) < 3:
            self.reason = "measuring_stability"
            return False
        spans = [max(item[1][i] for item in self.history) -
                 min(item[1][i] for item in self.history) for i in range(3)]
        self.metrics.update(dict(zip(("span_y", "span_u", "span_v"), spans)))
        if min(item[1][0] for item in self.history) < self.min_luma:
            self.reason = "image_too_dark"
        elif spans[0] > self.luma_tolerance:
            self.reason = "brightness_changing"
        elif max(spans[1:]) > self.chroma_tolerance:
            self.reason = "color_changing"
        else:
            self.reason = "stable"
            self.ready_ns = now_ns
            self.ready_frame_idx = frame_idx
            self.ready_metrics = dict(self.metrics)
            self.history.clear()
            return True
        return False

    def status(self, now_ns):
        age = None if self.last_ns is None else (now_ns - self.last_ns) / 1e6
        fresh = age is not None and age <= 1000
        return {
            "ready": self.ready_ns is not None and fresh,
            "reason": self.reason if fresh else "waiting_for_fresh_frames",
            "frames_seen": self.count, "last_frame_age_ms": age,
            "ready_mono_ns": self.ready_ns,
            "ready_stream_frame_idx": self.ready_frame_idx,
            "stability_at_ready": self.ready_metrics,
            "warmup_seconds": self.warmup_seconds,
            "stable_seconds": self.stable_seconds,
            "min_frames": self.min_frames,
            "luma_tolerance": self.luma_tolerance,
            "chroma_tolerance": self.chroma_tolerance,
            "min_luma": self.min_luma,
            **self.metrics,
        }


@dataclass(order=True)
class Job:
    execute_at_ns: int
    job_id: str = field(compare=False)
    command: dict = field(compare=False)
    clock_mode: str = field(default="monotonic", compare=False)
    scheduled_clock_ns: int = field(default=0, compare=False)
    committed: bool = field(default=False, compare=False)
    owner: Optional["ClientSession"] = field(default=None, compare=False)


class V4L2Recorder:
    def __init__(
        self, device, width, height, fps, input_format, output_pix_fmt,
        output_dir, showinfo, controls=None, control_strict=True,
        ready_frames=90, ring_frames=300, color_range="pc",
        colorspace="bt709", color_primaries="bt709", color_trc="bt709",
        in_range="pc", in_color_matrix="bt709", warmup_seconds=5.0,
        stable_seconds=2.0, luma_tolerance=3.0, chroma_tolerance=2.0,
        min_luma=25.0
    ):
        self.ffmpeg = require_executable("ffmpeg")
        self.v4l2_ctl = shutil.which("v4l2-ctl")
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.input_format = input_format
        self.output_pix_fmt = output_pix_fmt
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.showinfo = showinfo
        self.ready_frames = max(1, int(ready_frames))
        self.ring_frames = max(2, int(ring_frames))
        self.readiness = FrameReadiness(
            self.ready_frames, warmup_seconds, stable_seconds,
            luma_tolerance, chroma_tolerance, min_luma,
        )
        self.color_range = color_range
        self.colorspace = colorspace
        self.color_primaries = color_primaries
        self.color_trc = color_trc
        self.in_range = in_range
        self.in_color_matrix = in_color_matrix
        self.frame_size = self._expected_size(width, height, output_pix_fmt, 1)
        if self.frame_size is None:
            raise RuntimeError(f"unsupported raw output pixel format: {output_pix_fmt}")
        self.cv = threading.Condition()
        self.frames = deque(maxlen=self.ring_frames)
        self.frame_index = 0
        self.stream_error = None
        self.stderr_lines = deque(maxlen=1000)
        self.proc = None
        self.reader_thread = None
        self.stderr_thread = None
        self.stopping = threading.Event()
        if not Path(device).exists():
            raise RuntimeError(
                f"V4L2 device not found: {device}. Check `v4l2-ctl --list-devices`."
            )
        self.applied_controls = []
        try:
            self.apply_controls(controls or [], control_strict)
            self._start_stream()
        except Exception:
            self.close()
            raise

    def _stream_command(self):
        scale_filter = (
            f"scale=in_range={self.in_range}:out_range={self.color_range}:"
            f"in_color_matrix={self.in_color_matrix}:out_color_matrix={self.colorspace}"
        )
        vf = f"{scale_filter},format={self.output_pix_fmt}"
        return [
            self.ffmpeg,
            "-hide_banner",
            "-loglevel", "warning",
            "-nostdin",
            "-f", "v4l2",
            "-thread_queue_size", "8",
            "-framerate", str(self.fps),
            "-video_size", f"{self.width}x{self.height}",
            "-input_format", self.input_format,
            "-use_wallclock_as_timestamps", "1",
            "-i", self.device,
            "-an",
            "-vf", vf,
            "-c:v", "rawvideo",
            "-vsync", "0",
            "-pix_fmt", self.output_pix_fmt,
            "-color_range", self.color_range,
            "-colorspace", self.colorspace,
            "-color_primaries", self.color_primaries,
            "-color_trc", self.color_trc,
            "-f", "rawvideo",
            "pipe:1",
        ]

    def _start_stream(self):
        cmd = self._stream_command()
        self.proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdin=subprocess.DEVNULL,
            bufsize=0,
        )
        self.stream_command = cmd
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.stderr_thread = threading.Thread(target=self._stderr_loop, daemon=True)
        self.reader_thread.start()
        self.stderr_thread.start()
        print("Opening V4L2/ffmpeg stream and warming camera...", flush=True)

    def _stderr_loop(self):
        if self.proc is None or self.proc.stderr is None:
            return
        while not self.stopping.is_set():
            line = self.proc.stderr.readline()
            if not line:
                break
            try:
                text = line.decode("utf-8", errors="replace").rstrip()
            except AttributeError:
                text = str(line).rstrip()
            with self.cv:
                self.stderr_lines.append(text)
                if "driver changed" in text.lower():
                    self.stream_error = text
                self.cv.notify_all()

    def _read_exact_frame(self):
        if self.proc is None or self.proc.stdout is None:
            return None
        chunks = []
        remaining = self.frame_size
        while remaining > 0 and not self.stopping.is_set():
            chunk = self.proc.stdout.read(remaining)
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks) if remaining == 0 else None

    def _reader_loop(self):
        try:
            while not self.stopping.is_set():
                data = self._read_exact_frame()
                if data is None:
                    break
                read_mono_ns = time.monotonic_ns()
                metrics = FrameReadiness.measure(
                    data, self.width, self.height, self.output_pix_fmt
                )
                with self.cv:
                    became_ready = self.readiness.observe(
                        self.frame_index, read_mono_ns, metrics
                    )
                    if became_ready:
                        self.frames.clear()
                    frame = {
                        "frame_idx": self.frame_index,
                        "read_mono_ns": read_mono_ns,
                        "data": data,
                        "image_metrics": metrics,
                    }
                    # Never retain startup frames as capture candidates.
                    if self.readiness.ready_ns is not None and not became_ready:
                        self.frames.append(frame)
                    self.frame_index += 1
                    self.cv.notify_all()
                if became_ready:
                    print(f"CAMERA READY: {self.device}; "
                          f"discarded {self.frame_index} startup frames; "
                          f"Y/U/V={tuple(round(v, 2) for v in metrics)}", flush=True)
        except Exception as exc:
            with self.cv:
                self.stream_error = str(exc)
                self.cv.notify_all()
        finally:
            with self.cv:
                if self.stream_error is None and not self.stopping.is_set():
                    self.stream_error = "ffmpeg stream ended"
                self.cv.notify_all()

    def camera_status(self):
        with self.cv:
            status = self.readiness.status(time.monotonic_ns())
            if self.stream_error or self.stopping.is_set():
                status.update(ready=False, reason=self.stream_error or "stopped")
            status["stream_error"] = self.stream_error
            return status

    def require_ready(self):
        status = self.camera_status()
        if not status["ready"]:
            raise RuntimeError(f"camera_not_ready: {status['reason']}")
        return status

    def wait_ready(self, timeout=60.0):
        deadline = time.monotonic() + timeout
        with self.cv:
            while not self.camera_status()["ready"] and self.stream_error is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.cv.wait(remaining)
            return self.camera_status()["ready"]

    def apply_controls(self, controls, strict=True):
        if not controls:
            return
        if self.v4l2_ctl is None:
            message = "v4l2-ctl is required for camera controls but was not found"
            if strict:
                raise RuntimeError(message)
            print(f"WARNING: {message}", flush=True)
            return
        for name, value in controls:
            cmd = [
                self.v4l2_ctl, "-d", self.device,
                f"--set-ctrl={name}={value}",
            ]
            result = subprocess.run(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            if result.returncode != 0:
                message = (
                    f"failed to set V4L2 control {name}={value}: "
                    f"{result.stderr.strip() or result.stdout.strip()}"
                )
                if strict:
                    raise RuntimeError(message)
                print(f"WARNING: {message}", flush=True)
            else:
                self.applied_controls.append((name, value))
                print(f"Set V4L2 control: {name}={value}", flush=True)

    def capture(self, job):
        # Never wait for warmup here: delaying an individual camera would
        # invalidate the common timestamp chosen by the master.
        ready_status = self.require_ready()
        self.capture_ready_status = ready_status
        if job.execute_at_ns <= ready_status["ready_mono_ns"]:
            raise RuntimeError("capture timestamp precedes camera readiness")
        command = job.command
        device = command.get("device") or self.device
        width = int(command.get("width", self.width))
        height = int(command.get("height", self.height))
        fps = float(command.get("fps", self.fps))
        input_format = command.get("input_format", self.input_format)
        output_pix_fmt = command.get("output_pix_fmt", self.output_pix_fmt)
        color_range = command.get("color_range", "pc")
        colorspace = command.get("colorspace", "bt709")
        color_primaries = command.get("color_primaries", "bt709")
        color_trc = command.get("color_trc", "bt709")
        in_range = command.get("in_range", color_range)
        in_color_matrix = command.get("in_color_matrix", colorspace)
        target_frames = int(command.get("target_frames", command.get("frame_count", 1)))
        if target_frames < 1:
            raise ValueError("target_frames must be positive")
        frame_count = target_frames
        if device != self.device:
            raise RuntimeError(
                f"persistent stream is already open on {self.device}; "
                f"cannot switch to {device} during capture"
            )
        if (width, height, float(fps), input_format, output_pix_fmt) != (
            self.width, self.height, float(self.fps), self.input_format, self.output_pix_fmt
        ):
            raise RuntimeError(
                "capture parameters must match the already-open camera stream: "
                f"stream={self.width}x{self.height}@{self.fps} "
                f"{self.input_format}->{self.output_pix_fmt}, "
                f"requested={width}x{height}@{fps} {input_format}->{output_pix_fmt}"
            )
        fallback_date = datetime.today().strftime("%Y%m%d")
        session_date = safe_name(command.get("session_date"), fallback_date)
        session_time = safe_name(command.get("session_time"), datetime.today().strftime("%H%M"))
        session_stamp = safe_name(
            command.get("session_stamp"),
            f"{session_date}_{session_time}",
        )
        directory = self.output_dir / session_date
        directory.mkdir(parents=True, exist_ok=True)
        ext = PIX_FMT_EXT.get(output_pix_fmt, "yuv")
        fps_label = self._fps_label(fps)
        format_label = PIX_FMT_LABEL.get(output_pix_fmt, output_pix_fmt)
        hostname = socket.gethostname()
        filename = command.get(
            "filename",
            f"{hostname}_{width}x{height}_{format_label}_{fps_label}_{session_stamp}.{ext}",
        )
        video_path = directory / filename
        csv_path = Path(str(video_path) + ".csv")
        log_path = Path(str(video_path) + ".ffmpeg.log")
        meta_path = Path(str(video_path) + ".meta.json")

        capture_timeout = float(command.get("capture_timeout", 60.0))
        scheduled_mono_ns = int(job.execute_at_ns)
        start_ns = time.monotonic_ns()
        rows = []
        success, reason = True, "video_saved"
        try:
            with video_path.open("wb") as video_file:
                self._save_frames(video_file, scheduled_mono_ns, ready_status,
                                  target_frames, capture_timeout, rows)
        except Exception as exc:
            success, reason = False, str(exc)

        return self._finish_report(
            success, reason, video_path, csv_path, log_path,
            self.stream_command, list(self.stderr_lines), start_ns,
            time.monotonic_ns(), job, width, height, frame_count,
            target_frames, output_pix_fmt, meta_path, color_range, colorspace,
            color_primaries, color_trc, in_range, in_color_matrix, rows,
        )

    def _save_frames(self, video_file, scheduled_mono_ns, ready_status,
                     target_frames, capture_timeout, rows):
        deadline = time.monotonic() + capture_timeout
        next_idx = ready_status["ready_stream_frame_idx"] + 1
        while len(rows) < target_frames:
            with self.cv:
                while True:
                    status = self.camera_status()
                    if (not status["ready"] or
                            status["ready_mono_ns"] != ready_status["ready_mono_ns"]):
                        raise RuntimeError(f"camera lost readiness: {status['reason']}")
                    frame = next((f for f in self.frames
                                  if f["frame_idx"] >= next_idx and
                                  f["read_mono_ns"] >= scheduled_mono_ns), None)
                    if frame is not None:
                        if rows and frame["frame_idx"] != next_idx:
                            raise RuntimeError("capture_buffer_overrun: writer lost frames")
                        # If the ring no longer covers the scheduled gate, we
                        # cannot claim that the first requested frame survived.
                        if (not rows and self.frames[0]["read_mono_ns"] > scheduled_mono_ns
                                and len(self.frames) == self.frames.maxlen):
                            raise RuntimeError("capture_buffer_overrun: start frame lost")
                        next_idx = frame["frame_idx"] + 1
                        break
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise RuntimeError("capture_timeout")
                    self.cv.wait(min(.1, remaining))
            video_file.write(frame["data"])
            rows.append({
                "frame_idx": len(rows), "stream_frame_idx": frame["frame_idx"],
                "read_mono_ns": frame["read_mono_ns"],
                "image_metrics": frame["image_metrics"],
                "timestamp_error_ms": (frame["read_mono_ns"] - scheduled_mono_ns) / 1e6,
            })

    def _finish_report(
        self, success, reason, video_path, csv_path, log_path, ffmpeg_cmd,
        stderr_lines, start_ns, end_ns, job, width, height, frame_count,
        target_frames, output_pix_fmt, meta_path, color_range, colorspace,
        color_primaries, color_trc, in_range, in_color_matrix, rows=None
    ):
        rows = rows or []
        size_bytes = video_path.stat().st_size if video_path.exists() else 0
        expected_size = self._expected_size(width, height, output_pix_fmt, target_frames)
        if success and (len(rows) != target_frames or size_bytes != expected_size):
            success = False
            reason = f"unexpected_capture_size frames={len(rows)}/{target_frames} bytes={size_bytes}/{expected_size}"
        log_path.write_text("\n".join(stderr_lines), encoding="utf-8", errors="replace")
        meta = {
            "success": success,
            "reason": reason,
            "camera_readiness": self.capture_ready_status,
            "applied_controls": dict(self.applied_controls),
            "target_frames": target_frames,
            "saved_frames": len(rows),
            "file": str(video_path),
            "session_date": job.command.get("session_date"),
            "session_time": job.command.get("session_time"),
            "session_stamp": job.command.get("session_stamp"),
            "width": width,
            "height": height,
            "fps": job.command.get("fps", self.fps),
            "input_format": job.command.get("input_format", self.input_format),
            "output_pix_fmt": output_pix_fmt,
            "bit_depth": 8,
            "raw_layout": "I420" if output_pix_fmt == "yuv420p" else output_pix_fmt,
            "color_range": color_range,
            "colorspace": colorspace,
            "color_primaries": color_primaries,
            "color_trc": color_trc,
            "in_range": in_range,
            "in_color_matrix": in_color_matrix,
            "note": (
                "Raw .yuv does not embed color metadata; this sidecar records "
                "the intended interpretation and ffmpeg conversion settings."
            ),
        }
        meta_path.write_text(
            json.dumps(meta, indent=2, sort_keys=True), encoding="utf-8"
        )
        size_bytes = video_path.stat().st_size if video_path.exists() else 0
        with csv_path.open("w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "frame_idx", "stream_frame_idx", "frame_read_mono_ns",
                "scheduled_ns", "scheduled_mono_ns", "capture_start_ns",
                "capture_end_ns", "start_late_ms", "timestamp_error_ms",
                "width", "height", "output_pix_fmt", "file_size_bytes",
                "mean_y", "mean_u", "mean_v",
            ])
            if rows:
                for row in rows:
                    writer.writerow([
                        row["frame_idx"], row["stream_frame_idx"],
                        row["read_mono_ns"], job.scheduled_clock_ns,
                        job.execute_at_ns, start_ns, end_ns,
                        (start_ns - job.execute_at_ns) / 1e6,
                        row["timestamp_error_ms"],
                        width, height, output_pix_fmt, size_bytes,
                        *row["image_metrics"],
                    ])
            else:
                writer.writerow([
                    "", "", "", job.scheduled_clock_ns, job.execute_at_ns,
                    start_ns, end_ns,
                    (start_ns - job.execute_at_ns) / 1e6,
                    "",
                    width, height, output_pix_fmt, size_bytes,
                    "", "", "",
                ])

        expected_min = self._expected_size(width, height, output_pix_fmt, frame_count)
        if success and expected_min is not None and size_bytes != expected_min:
            success = False
            reason = f"unexpected_file_size expected={expected_min} got={size_bytes}"

        return {
            "success": success,
            "reason": reason,
            "capture": {
                "filename": str(video_path),
                "csv_filename": str(csv_path),
                "log_filename": str(log_path),
                "meta_filename": str(meta_path),
                "target_frames": target_frames,
                "frame_count": frame_count,
                "showinfo_frames": len(rows),
                "saved_frames": len(rows),
                "camera_ready": self.camera_status()["ready"],
                "startup_frames_discarded": self.capture_ready_status["ready_stream_frame_idx"] + 1,
                "file_size_bytes": size_bytes,
                "ffmpeg_start_ns": start_ns,
                "ffmpeg_end_ns": end_ns,
                "start_late_ms": (start_ns - job.execute_at_ns) / 1e6,
                "command": " ".join(ffmpeg_cmd),
            },
        }

    @staticmethod
    def _expected_size(width, height, pix_fmt, frames):
        if pix_fmt == "yuyv422":
            return width * height * 2 * frames
        if pix_fmt in ("yuv420p", "nv12"):
            return width * height * 3 // 2 * frames
        if pix_fmt in ("rgb24", "bgr24"):
            return width * height * 3 * frames
        return None

    @staticmethod
    def _fps_label(fps):
        fps_float = float(fps)
        if fps_float.is_integer():
            return f"{int(fps_float)}fps"
        return f"{str(fps_float).replace('.', 'p')}fps"

    def close(self):
        self.stopping.set()
        if self.proc is not None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=2.0)
            except Exception:
                try:
                    self.proc.kill()
                    self.proc.wait(timeout=2.0)
                except Exception:
                    pass
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)
        if self.stderr_thread is not None:
            self.stderr_thread.join(timeout=1.0)
        if self.proc is not None:
            for pipe in (self.proc.stdout, self.proc.stderr):
                if pipe is not None:
                    pipe.close()


class Scheduler:
    def __init__(self, recorder):
        self.recorder = recorder
        self.cv = threading.Condition()
        self.jobs = {}
        self.heap = []
        self.stopping = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def prepare(self, job_id, command, execute_at_ns, owner, clock_mode="monotonic"):
        import heapq

        parsed = json.loads(command)
        self.recorder.require_ready()
        scheduled_clock_ns = int(execute_at_ns)
        now = time.monotonic_ns()
        if clock_mode == "tai":
            execute_at_ns = now + (scheduled_clock_ns - clock_now_ns("tai"))
        elif clock_mode != "monotonic":
            raise ValueError(f"unsupported execution clock: {clock_mode}")
        job = Job(
            int(execute_at_ns), job_id, parsed, clock_mode,
            scheduled_clock_ns, False, owner
        )
        with self.cv:
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
        import heapq

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
            self._execute(job)

    def _execute(self, job):
        actual_ns = clock_now_ns(job.clock_mode)
        try:
            verification = self.recorder.capture(job)
        except Exception as exc:
            verification = {"success": False, "reason": str(exc), "capture": {}}
        job.owner.send({
            "type": "executed", "job_id": job.job_id,
            "scheduled_ns": job.scheduled_clock_ns,
            "actual_ns": actual_ns,
            "late_ns": actual_ns - job.scheduled_clock_ns,
            "verification": verification,
        })

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
                        self.send({
                            "type": "hello", "version": PROTOCOL_VERSION,
                            "robot": socket.gethostname(), "dry_run": False,
                            "tai_available": tai_available(),
                        })
                    elif kind == "sync":
                        clock_mode = message.get("clock_mode", "monotonic")
                        t1_ns = received_ns if clock_mode == "monotonic" else clock_now_ns(clock_mode)
                        self.send({
                            "type": "sync", "seq": message["seq"],
                            "t0_ns": message["t0_ns"], "t1_ns": t1_ns,
                            "t2_ns": clock_now_ns(clock_mode),
                            "clock_mode": clock_mode,
                        })
                    elif kind == "camera_status":
                        self.send({"type": "camera_status",
                                   **self.scheduler.recorder.camera_status()})
                    elif kind == "prepare":
                        self.scheduler.prepare(
                            message["job_id"], message["command"],
                            int(message["execute_at_ns"]), self,
                            message.get("clock_mode", "monotonic"),
                        )
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
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--params", default=str(Path(__file__).resolve().with_name("input_parameters.txt")))
    pre_args, _ = pre_parser.parse_known_args()
    params = parse_parameter_file(pre_args.params)
    slave = params.get("slave", {})

    parser = argparse.ArgumentParser(description=__doc__, parents=[pre_parser])
    parser.add_argument("--listen", default=slave.get("listen", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(slave.get("port", DEFAULT_PORT)))
    parser.add_argument("--device", default=slave.get("device", "/dev/video6"))
    parser.add_argument("--width", type=int, default=int(slave.get("width", 1920)))
    parser.add_argument("--height", type=int, default=int(slave.get("height", 1080)))
    parser.add_argument("--fps", type=float, default=float(slave.get("fps", 30.0)))
    parser.add_argument("--input-format", default=slave.get("input_format", "yuyv422"),
                        help="V4L2 input format, e.g. yuyv422 or mjpeg")
    parser.add_argument("--output-pix-fmt", default=slave.get("output_pix_fmt", "yuv420p"),
                        choices=sorted(PIX_FMT_EXT))
    parser.add_argument("--output-dir", default=slave.get("output_dir", "./Results"))
    parser.add_argument("--no-showinfo", action="store_true",
                        default=parse_bool(slave.get("no_showinfo", False)),
                        help="disable per-frame ffmpeg showinfo CSV parsing")
    parser.add_argument("--keep-alive", action="store_true",
                        default=parse_bool(slave.get("keep_alive", False)),
                        help="keep listening after the master disconnects")
    parser.add_argument("--control-strict", action="store_true",
                        default=parse_bool(slave.get("control_strict", True)),
                        help="fail if any V4L2 control cannot be applied")
    parser.add_argument("--no-control-strict", action="store_false",
                        dest="control_strict",
                        help="warn instead of failing when a V4L2 control fails")
    parser.add_argument("--ready-frames", type=int,
                        default=int(slave.get("ready_frames", 90)),
                        help="minimum startup frames to discard (also requires image stability)")
    for name, default in (("warmup_seconds", 5.0), ("stable_seconds", 2.0),
                          ("luma_tolerance", 3.0), ("chroma_tolerance", 2.0),
                          ("min_luma", 25.0)):
        parser.add_argument("--" + name.replace("_", "-"), type=float,
                            default=float(slave.get(name, default)))
    parser.add_argument("--ring-frames", type=int,
                        default=int(slave.get("ring_frames", 300)),
                        help="number of most recent live frames to keep in RAM")
    parser.add_argument("--color-range", default=slave.get("color_range", "pc"))
    parser.add_argument("--colorspace", default=slave.get("colorspace", "bt709"))
    parser.add_argument("--color-primaries", default=slave.get("color_primaries", "bt709"))
    parser.add_argument("--color-trc", default=slave.get("color_trc", "bt709"))
    parser.add_argument("--in-range", default=slave.get("in_range", "pc"))
    parser.add_argument("--in-color-matrix", default=slave.get("in_color_matrix", "bt709"))
    args = parser.parse_args()
    if args.width < 2 or args.height < 2 or args.width % 2 or args.height % 2:
        parser.error("width and height must be positive even numbers")
    if not math.isfinite(args.fps) or args.fps <= 0:
        parser.error("fps must be finite and positive")
    print(f"Parameters: {Path(args.params).resolve()}", flush=True)
    args.camera_controls = camera_controls(params)
    return args


def main():
    args = parse_args()
    recorder = V4L2Recorder(
        args.device, args.width, args.height, args.fps,
        args.input_format, args.output_pix_fmt, args.output_dir,
        not args.no_showinfo, args.camera_controls, args.control_strict,
        args.ready_frames, args.ring_frames, args.color_range,
        args.colorspace, args.color_primaries, args.color_trc,
        args.in_range, args.in_color_matrix,
        args.warmup_seconds, args.stable_seconds,
        args.luma_tolerance, args.chroma_tolerance, args.min_luma,
    )
    scheduler = Scheduler(recorder)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.listen, args.port))
    server.listen()
    server.settimeout(0.5)
    shutdown_event = threading.Event()
    print(f"slave_realsense_v4l2 listening on {args.listen}:{args.port}", flush=True)
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
        print("Stopping slave_realsense_v4l2", flush=True)
    finally:
        server.close()
        scheduler.close()
        recorder.close()


if __name__ == "__main__":
    main()
